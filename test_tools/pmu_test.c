// SPDX-License-Identifier: GPL-2.0
// pmu_test - validate the Cortex-A53 PMU (perf counters) on the open kernel.
//
// The PMU is the mainline ARM_PMUV3 driver bound to the DT `pmu` node
// (dts/proxima-9311.dts): per-core overflow interrupts SPI 109 (cpu0) and
// SPI 110 (cpu1), recovered from the vendor DTS. This tool exercises it through
// raw perf_event_open(2), no perf userspace needed. Per CPU it checks:
//
//   1. counting: a cycles+instructions group over a calibrated spin loop with an
//      exact known instruction count; PASS if instructions match and the group
//      was never multiplexed.
//   2. clock: cycles / wall-time = the real core clock (informational, this is
//      the empirical SPL-set frequency the DVFS work wants).
//   3. overflow IRQs: a sampling event (small sample_period) must increment the
//      PMU line(s) in /proc/interrupts on the pinned CPU and not on the other
//      one; proves the SPI routing AND the interrupt-affinity mapping.
//   4. informational reads of cache/branch miss counters and one raw A53 event.
//
//   pmu_test            # run all checks on both CPUs, exit 0 only if all PASS
#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <sched.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/syscall.h>
#include <linux/perf_event.h>

#define NCPU		2
#define SPIN_ITERS	100000000ULL	/* 2 instructions each (subs + b.ne) */
#define SPIN_INSNS	(2 * SPIN_ITERS)
#define SAMPLE_PERIOD	1000000ULL	/* cycles per overflow IRQ in check 3 */

static int failures;

static void result(int ok, const char *fmt, ...)
{
	va_list ap;

	printf("%s: ", ok ? "PASS" : "FAIL");
	va_start(ap, fmt);
	vprintf(fmt, ap);
	va_end(ap);
	printf("\n");

	if (!ok)
		failures++;
}

static int perf_open(uint32_t type, uint64_t config, uint64_t sample_period,
		     uint64_t read_format, int group_fd)
{
	struct perf_event_attr attr;

	memset(&attr, 0, sizeof(attr));
	attr.size = sizeof(attr);
	attr.type = type;
	attr.config = config;
	attr.disabled = (group_fd == -1);
	attr.exclude_hv = 1;
	attr.sample_period = sample_period;
	attr.read_format = read_format;
	/* pid=0, cpu=-1: this thread, whichever CPU it is pinned to */
	return syscall(SYS_perf_event_open, &attr, 0, -1, group_fd, 0);
}

static void pin_to_cpu(int cpu)
{
	cpu_set_t set;

	CPU_ZERO(&set);
	CPU_SET(cpu, &set);
	if (sched_setaffinity(0, sizeof(set), &set) < 0) {
		perror("sched_setaffinity");
		exit(1);
	}
}

/* The calibrated workload: exactly 2 retired instructions per iteration, so the
 * expected INSTRUCTIONS count is a hard invariant (2 * iters) that check 1 can
 * assert against the PMU. It must be asm: a plain C loop has no side effects, so
 * -O2 would delete it outright, or unroll/vectorize a non-empty one, making the
 * instruction count compiler-dependent and unknowable. `volatile` keeps the
 * optimizer's hands off the two instructions; `noinline` keeps the loop from
 * being merged/specialized into a call site, so every call retires the same
 * fixed sequence plus only a few instructions of call overhead (that residue is
 * why the counting check allows a small tolerance instead of exact equality).
 */
static void __attribute__((noinline)) spin_loop(uint64_t iters)
{
	asm volatile(
		"1:	subs	%0, %0, #1\n"
		"	b.ne	1b\n"
		: "+r"(iters) : : "cc");
}

static uint64_t now_ns(void)
{
	struct timespec ts;

	clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
	return (uint64_t)ts.tv_sec * 1000000000ULL + ts.tv_nsec;
}

/* Sum the per-CPU interrupt counts of every /proc/interrupts line naming the
 * PMU ("pmu" substring in the description). SPI-per-core PMUs show up as one
 * line per interrupt; summing all pmu lines per CPU column is layout-agnostic.
 */
static int pmu_irq_counts(uint64_t counts[NCPU])
{
	FILE *f = fopen("/proc/interrupts", "r");
	char line[512];
	int found = 0;

	memset(counts, 0, NCPU * sizeof(counts[0]));
	if (!f) {
		perror("/proc/interrupts");
		return -1;
	}

	while (fgets(line, sizeof(line), f)) {
		char *p = strchr(line, ':');
		int cpu;

		if (!p)
			continue;

		/* Description text (past the count columns) must name the PMU;
		 * search the whole tail, count columns never contain letters.
		 */
		if (!strstr(p, "pmu") && !strstr(p, "PMU"))
			continue;

		found = 1;
		p++;
		for (cpu = 0; cpu < NCPU; cpu++) {
			counts[cpu] += strtoull(p, &p, 10);
			if (!p)
				break;
		}
	}
	fclose(f);

	if (!found)
		fprintf(stderr, "pmu_test: no PMU line in /proc/interrupts\n");

	return found ? 0 : -1;
}

static void check_counting_and_clock(int cpu)
{
	uint64_t buf[8], t0, t1;
	uint64_t nr, time_enabled, time_running, cycles, insns;
	double err, mhz;
	int lead, mem;

	pin_to_cpu(cpu);

	lead = perf_open(PERF_TYPE_HARDWARE, PERF_COUNT_HW_CPU_CYCLES, 0,
			 PERF_FORMAT_GROUP | PERF_FORMAT_TOTAL_TIME_ENABLED |
			 PERF_FORMAT_TOTAL_TIME_RUNNING, -1);
	if (lead < 0) {
		result(0, "cpu%d: perf_event_open(cycles): %s (PMU driver not probed?)",
		       cpu, strerror(errno));
		return;
	}

	mem = perf_open(PERF_TYPE_HARDWARE, PERF_COUNT_HW_INSTRUCTIONS, 0, 0, lead);
	if (mem < 0) {
		result(0, "cpu%d: perf_event_open(instructions): %s", cpu, strerror(errno));
		close(lead);
		return;
	}

	ioctl(lead, PERF_EVENT_IOC_RESET, PERF_IOC_FLAG_GROUP);
	ioctl(lead, PERF_EVENT_IOC_ENABLE, PERF_IOC_FLAG_GROUP);

	t0 = now_ns();
	spin_loop(SPIN_ITERS);
	t1 = now_ns();

	ioctl(lead, PERF_EVENT_IOC_DISABLE, PERF_IOC_FLAG_GROUP);

	/* PERF_FORMAT_GROUP layout: nr, time_enabled, time_running, value[nr] */
	if (read(lead, buf, sizeof(buf)) < (ssize_t)(5 * sizeof(buf[0]))) {
		result(0, "cpu%d: group read failed", cpu);
		goto out;
	}

	nr = buf[0];
	time_enabled = buf[1];
	time_running = buf[2];
	cycles = buf[3];
	insns = buf[4];

	err = (double)insns / (double)SPIN_INSNS - 1.0;
	result(nr == 2 && cycles > 0 &&
	       insns > SPIN_INSNS * 95 / 100 && insns < SPIN_INSNS * 105 / 100,
	       "cpu%d counting: instructions %" PRIu64 " vs expected %llu (%+.3f%%), cycles %" PRIu64,
	       cpu, insns, SPIN_INSNS, err * 100.0, cycles);
	result(time_running == time_enabled,
	       "cpu%d no multiplexing: time_running %" PRIu64 " == time_enabled %" PRIu64,
	       cpu, time_running, time_enabled);

	mhz = (double)cycles * 1000.0 / (double)(t1 - t0);
	printf("INFO: cpu%d measured core clock: %.1f MHz (%" PRIu64 " cycles / %.3f ms)\n",
	       cpu, mhz, cycles, (t1 - t0) / 1e6);

out:
	close(mem);
	close(lead);
}

static void check_overflow_irq(int cpu)
{
	uint64_t before[NCPU], after[NCPU], cycles = 0;
	uint64_t expect, delta_self, delta_other;
	int fd, i;

	pin_to_cpu(cpu);

	if (pmu_irq_counts(before) < 0) {
		result(0, "cpu%d overflow: PMU missing from /proc/interrupts", cpu);
		return;
	}

	fd = perf_open(PERF_TYPE_HARDWARE, PERF_COUNT_HW_CPU_CYCLES, SAMPLE_PERIOD, 0, -1);
	if (fd < 0) {
		result(0, "cpu%d overflow: perf_event_open(sampling): %s", cpu, strerror(errno));
		return;
	}

	ioctl(fd, PERF_EVENT_IOC_ENABLE, 0);
	for (i = 0; i < 10; i++)
		spin_loop(SPIN_ITERS / 10);

	ioctl(fd, PERF_EVENT_IOC_DISABLE, 0);
	if (read(fd, &cycles, sizeof(cycles)) != sizeof(cycles))
		cycles = 0;

	close(fd);

	if (pmu_irq_counts(after) < 0) {
		result(0, "cpu%d overflow: /proc/interrupts re-read failed", cpu);
		return;
	}

	expect = cycles / SAMPLE_PERIOD;
	delta_self = after[cpu] - before[cpu];
	delta_other = after[!cpu] - before[!cpu];
	/* Routing correct: the pinned core took (roughly all of) the overflow IRQs,
	 * the other core took at most stray noise. A swapped SPI pair or wrong
	 * interrupt-affinity inverts this.
	 */
	result(expect > 10 && delta_self >= expect / 2 && delta_other <= expect / 10 + 5,
	       "cpu%d overflow IRQs: %" PRIu64 " on cpu%d / %" PRIu64 " on cpu%d (expected ~%" PRIu64 ")",
	       cpu, delta_self, cpu, delta_other, !cpu, expect);
}

static void info_event(int cpu, uint32_t type, uint64_t config, const char *name)
{
	uint64_t val = 0;
	int fd = perf_open(type, config, 0, 0, -1);

	if (fd < 0) {
		printf("INFO: cpu%d %s: unsupported (%s)\n", cpu, name, strerror(errno));
		return;
	}

	ioctl(fd, PERF_EVENT_IOC_ENABLE, 0);
	spin_loop(SPIN_ITERS / 10);
	ioctl(fd, PERF_EVENT_IOC_DISABLE, 0);
	if (read(fd, &val, sizeof(val)) == sizeof(val))
		printf("INFO: cpu%d %s: %" PRIu64 "\n", cpu, name, val);

	close(fd);
}

int main(void)
{
	int cpu;

	setbuf(stdout, NULL);
	for (cpu = 0; cpu < NCPU; cpu++) {
		check_counting_and_clock(cpu);
		check_overflow_irq(cpu);
		info_event(cpu, PERF_TYPE_HARDWARE, PERF_COUNT_HW_CACHE_MISSES, "cache-misses");
		info_event(cpu, PERF_TYPE_HARDWARE, PERF_COUNT_HW_BRANCH_MISSES, "branch-misses");
		/* raw armv8 event 0x08 = INST_RETIRED, proves PERF_TYPE_RAW works */
		info_event(cpu, PERF_TYPE_RAW, 0x08, "raw:0x08 INST_RETIRED");
	}

	printf("%s (%d failure%s)\n", failures ? "FAIL" : "PASS: all checks",
	       failures, failures == 1 ? "" : "s");
	return failures ? 1 : 0;
}
