// SPDX-License-Identifier: GPL-2.0
/*
 * sd_rwtest - SD-card block read/write test through the block layer
 * (mmcblk -> mmc core -> dw_mci-artosyn), hang-safe.
 *
 * Each I/O phase runs in a forked child under a parent-enforced timeout so a
 * D-state mmc hang cannot wedge the test; on timeout the parent prints the
 * child's /proc/<pid>/wchan and exits 2.
 *
 * Test sequence (O_DIRECT unless -B):
 *   1. READ : read READ_LEN at offset 0 twice, compare.
 *   2. WRITE: at a scratch offset (default: 64 KiB-aligned, END_MARGIN below
 *      the device end):
 *        a. save the original contents to the backup file,
 *        b. write a per-sector tagged pattern, fsync,
 *        c. read back (always O_DIRECT), verify every byte,
 *        d. restore the original contents, fsync, read back, verify.
 *      On pass the backup file is deleted; on FAIL/HANG it is kept for
 *      `sd_rwtest restore <backup-file>`.
 *
 * The write test rewrites the scratch region in place; the region may hold
 * filesystem data.
 *
 * After a hang: the child is unkillable (D-state) and the mmc request queue
 * stays wedged - all further I/O to the card hangs until a power cycle.
 *
 * Usage:
 *   sd_rwtest [read|write|all]      # test selection (default all) on /dev/mmcblk1
 *   sd_rwtest -d <blockdev>         # other block device
 *   sd_rwtest -o <bytes>            # explicit scratch offset (4K-aligned)
 *   sd_rwtest -t <sec>              # per-phase timeout (default 20)
 *   sd_rwtest -b <file>             # backup file (default /run/sd_rwtest.bak)
 *   sd_rwtest -n <bytes>            # write-region size (default 64 KiB, 4K-multiple)
 *   sd_rwtest -B                    # buffered writes (page cache + fsync) instead
 *                                   # of O_DIRECT; verify reads stay O_DIRECT
 *   sd_rwtest restore <backup-file> # write a saved backup back to its offset
 *
 * Exit codes: 0 = PASS, 1 = data mismatch / I/O error, 2 = a phase HUNG,
 *             3 = usage/setup error.
 *
 * Build: aarch64-linux-gnu-gcc -static -O2 sd_rwtest.c -o sd_rwtest
 */
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <time.h>
#include <linux/fs.h>

#define READ_LEN	(4 * 1024 * 1024)  /* read-test length at offset 0 */
#define ALIGN		4096               /* O_DIRECT buffer/offset alignment */
#define END_MARGIN	(1024 * 1024)      /* scratch region distance below device end */
#define MAX_LEN		(16 * 1024 * 1024) /* write-region cap (also the restore cap) */

static const char *g_dev = "/dev/mmcblk1";
static const char *g_bak = "/run/sd_rwtest.bak";
static long long g_off = -1;		/* -1 = auto (near end of device) */
static int g_timeout = 20;		/* seconds per I/O phase */
static size_t g_len = 64 * 1024;	/* write-test region size (-n) */
static int g_buffered;			/* -B: buffered (page-cache) writes */

/* ---------- helpers ---------- */

static void *xalloc(size_t len)
{
	void *p = NULL;

	if (posix_memalign(&p, ALIGN, len) != 0) {
		fprintf(stderr, "posix_memalign(%zu): out of memory\n", len);
		exit(3);
	}

	return p;
}

static long long dev_size(const char *dev)
{
	unsigned long long sz = 0;
	int fd = open(dev, O_RDONLY);

	if (fd < 0) {
		fprintf(stderr, "open %s: %s\n", dev, strerror(errno));
		return -1;
	}

	if (ioctl(fd, BLKGETSIZE64, &sz) != 0) {
		fprintf(stderr, "BLKGETSIZE64 %s: %s\n", dev, strerror(errno));
		close(fd);
		return -1;
	}

	close(fd);
	return (long long)sz;
}

/* Print a pid's state letter and kernel wait channel. */
static void report_wchan(pid_t pid)
{
	char path[64], stat_state = '?', wchan[128] = "?";
	FILE *f;

	snprintf(path, sizeof(path), "/proc/%d/wchan", pid);
	f = fopen(path, "r");
	if (f) {
		size_t n = fread(wchan, 1, sizeof(wchan) - 1, f);

		wchan[n] = '\0';
		fclose(f);
	}
	snprintf(path, sizeof(path), "/proc/%d/stat", pid);

	f = fopen(path, "r");
	if (f) {
		/* field 3 (after "pid (comm)") is the state letter */
		char comm[64];
		int dummy;

		if (fscanf(f, "%d %63s %c", &dummy, comm, &stat_state) != 3)
			stat_state = '?';
		fclose(f);
	}

	printf("    child pid %d: state '%c', blocked in kernel at: %s\n",
	       pid, stat_state, wchan);
	if (strstr(wchan, "mmc_blk_rw_wait"))
		printf("    -> mmc_blk_rw_wait write hang\n");
}

/*
 * Run fn(arg) in a forked child. Returns the child's exit code, or -1 after
 * g_timeout seconds (a D-state child survives the SIGKILL and is left behind).
 */
static int run_phase(const char *name, int (*fn)(void *), void *arg)
{
	struct timespec t0, t1;
	pid_t pid;
	int waited_ms;

	printf("  [%s] ", name);
	fflush(stdout);
	clock_gettime(CLOCK_MONOTONIC, &t0);

	pid = fork();
	if (pid < 0) {
		perror("fork");
		exit(3);
	}

	if (pid == 0)
		_exit(fn(arg) == 0 ? 0 : 1);

	for (waited_ms = 0; waited_ms < g_timeout * 1000; waited_ms += 10) {
		int status;

		if (waitpid(pid, &status, WNOHANG) == pid) {
			int code = WIFEXITED(status) ? WEXITSTATUS(status) : 1;
			long ms;

			clock_gettime(CLOCK_MONOTONIC, &t1);
			ms = (t1.tv_sec - t0.tv_sec) * 1000 +
			     (t1.tv_nsec - t0.tv_nsec) / 1000000;
			printf("%s (%ld ms)\n", code == 0 ? "ok" : "FAILED", ms);

			return code;
		}
		usleep(10 * 1000);
	}

	printf("HUNG (no completion after %ds)\n", g_timeout);
	report_wchan(pid);
	kill(pid, SIGKILL);
	usleep(200 * 1000);
	waitpid(pid, NULL, WNOHANG);

	return -1;
}

/* Full read/write of exactly len bytes at off. Returns 0 or -1. */
static int do_io(int fd, void *buf, size_t len, long long off, int write_mode)
{
	char *p = buf;
	size_t done = 0;

	while (done < len) {
		ssize_t n = write_mode ?
			pwrite(fd, p + done, len - done, off + done) :
			pread(fd, p + done, len - done, off + done);

		if (n <= 0) {
			fprintf(stderr, "%s at +%zu: %s\n",
				write_mode ? "pwrite" : "pread", done,
				n == 0 ? "short" : strerror(errno));

			return -1;
		}
		done += n;
	}

	return 0;
}

/* Test pattern: every 512-byte sector tagged with its own sector number. */
static void fill_pattern(uint8_t *buf, size_t len, long long base_off)
{
	size_t s, i;

	for (s = 0; s < len; s += 512) {
		uint64_t *w = (uint64_t *)(buf + s);
		uint64_t tag = 0x5d7e577000000000ULL |
			       (uint64_t)((base_off + s) / 512);

		for (i = 0; i < 512 / 8; i++)
			w[i] = tag ^ (0x0101010101010101ULL * i);
	}
}

/* ---------- phases (each runs in its own timed child) ---------- */

struct io_task {
	long long off;
	size_t len;
	const uint8_t *expect;	/* verify read data against this (NULL = skip) */
	const uint8_t *data;	/* data to write (write phases) */
	const char *save_path;	/* save read data here (backup phase) */
};

static int phase_read_stable(void *arg)
{
	uint8_t *a, *b;
	int ret = -1;
	int fd = open(g_dev, O_RDONLY | O_DIRECT);

	(void)arg;
	if (fd < 0) {
		perror(g_dev);
		return -1;
	}

	a = xalloc(READ_LEN);
	b = xalloc(READ_LEN);
	if (do_io(fd, a, READ_LEN, 0, 0) == 0 &&
	    do_io(fd, b, READ_LEN, 0, 0) == 0) {
		if (memcmp(a, b, READ_LEN) == 0)
			ret = 0;
		else
			fprintf(stderr, "two reads of the same %d bytes differ\n",
				READ_LEN);
	}

	close(fd);
	free(a);
	free(b);

	return ret;
}

static int phase_backup(void *arg)
{
	struct io_task *t = arg;
	uint8_t *buf;
	int ret;
	int fd = open(g_dev, O_RDONLY | O_DIRECT);

	if (fd < 0) {
		perror(g_dev);
		return -1;
	}

	buf = xalloc(t->len);
	ret = do_io(fd, buf, t->len, t->off, 0);
	close(fd);

	if (ret == 0) {
		/* backup file layout: "SDRWBAK1" + u64 offset + u64 len + data */
		FILE *f = fopen(t->save_path, "w");
		uint64_t off = t->off, len = t->len;

		if (!f ||
		    fwrite("SDRWBAK1", 1, 8, f) != 8 ||
		    fwrite(&off, 8, 1, f) != 1 ||
		    fwrite(&len, 8, 1, f) != 1 ||
		    fwrite(buf, 1, t->len, f) != t->len ||
		    fclose(f) != 0) {
			fprintf(stderr, "saving backup %s: %s\n",
				t->save_path, strerror(errno));
			ret = -1;
		}
	}
	free(buf);
	return ret;
}

static int phase_write(void *arg)
{
	struct io_task *t = arg;
	int ret;
	int fd = open(g_dev, g_buffered ? O_WRONLY : (O_WRONLY | O_DIRECT));

	if (fd < 0) {
		perror(g_dev);
		return -1;
	}

	ret = do_io(fd, (void *)t->data, t->len, t->off, 1);
	if (ret == 0 && fsync(fd) != 0) {
		perror("fsync");
		ret = -1;
	}

	close(fd);
	return ret;
}

static int phase_verify(void *arg)
{
	struct io_task *t = arg;
	uint8_t *buf;
	int ret;
	int fd = open(g_dev, O_RDONLY | O_DIRECT);

	if (fd < 0) {
		perror(g_dev);
		return -1;
	}

	buf = xalloc(t->len);
	ret = do_io(fd, buf, t->len, t->off, 0);
	close(fd);
	if (ret == 0 && memcmp(buf, t->expect, t->len) != 0) {
		size_t i;

		for (i = 0; i < t->len && buf[i] == t->expect[i]; i++)
			;
		fprintf(stderr, "verify mismatch at offset +%zu (sector %lld)\n",
			i, (t->off + (long long)i) / 512);
		ret = -1;
	}
	free(buf);

	return ret;
}

/* ---------- tests ---------- */

static int test_read(void)
{
	int r;

	printf("READ test: %d MiB at offset 0, twice, compare (O_DIRECT)\n",
	       READ_LEN / (1024 * 1024));
	r = run_phase("read x2 + compare", phase_read_stable, NULL);
	printf("READ test: %s\n", r == 0 ? "PASS" : r < 0 ? "HUNG" : "FAIL");
	return r == 0 ? 0 : (r < 0 ? 2 : 1);
}

static int test_write(long long size)
{
	long long off = g_off;
	size_t len = g_len;
	uint8_t *pattern, *original;
	FILE *f;
	int r;

	if (off < 0)
		off = (size - END_MARGIN - (long long)len) &
		      ~(long long)(64 * 1024 - 1);

	if (off < 0 || off % ALIGN || off + (long long)len > size) {
		fprintf(stderr, "bad test offset 0x%llx (device is 0x%llx bytes)\n",
			off, size);
		return 3;
	}

	printf("WRITE test (%s): %zu KiB at offset 0x%llx (sector %lld), backup: %s\n",
	       g_buffered ? "buffered+fsync" : "O_DIRECT",
	       len / 1024, off, off / 512, g_bak);

	/* O_DIRECT requires aligned user buffers */
	pattern = xalloc(len);
	original = xalloc(len);
	fill_pattern(pattern, len, off);

	struct io_task bak = { .off = off, .len = len, .save_path = g_bak };

	if (run_phase("backup original", phase_backup, &bak) != 0) {
		printf("WRITE test: FAIL (could not back up the target region; not writing)\n");
		return 1;
	}

	/* reload the backup: the child's in-memory copy is gone after exit */
	f = fopen(g_bak, "r");
	if (!f || fseek(f, 24, SEEK_SET) != 0 ||
	    fread(original, 1, len, f) != len) {
		fprintf(stderr, "re-reading %s failed\n", g_bak);
		if (f)
			fclose(f);

		return 3;
	}
	fclose(f);

	struct io_task wr  = { .off = off, .len = len, .data = pattern };
	struct io_task vfy = { .off = off, .len = len, .expect = pattern };
	struct io_task rst = { .off = off, .len = len, .data = original };
	struct io_task vor = { .off = off, .len = len, .expect = original };

	r = run_phase("write pattern", phase_write, &wr);
	if (r != 0)
		goto hung_or_failed;

	r = run_phase("read-back verify", phase_verify, &vfy);
	if (r != 0)
		goto hung_or_failed;

	r = run_phase("restore original", phase_write, &rst);
	if (r != 0)
		goto hung_or_failed;

	r = run_phase("verify restore", phase_verify, &vor);
	if (r != 0)
		goto hung_or_failed;

	unlink(g_bak);
	printf("WRITE test: PASS (pattern written+verified, original restored+verified)\n");
	return 0;

hung_or_failed:
	printf("WRITE test: %s\n", r < 0 ? "HUNG" : "FAIL");
	printf("  original %zu KiB saved in %s (device offset 0x%llx).\n",
	       len / 1024, g_bak, off);
	if (r < 0)
		printf("  The mmc queue is likely wedged: power-cycle the goggle before any\n"
		       "  further SD I/O. After reboot, restore with:\n"
		       "    sd_rwtest -d %s restore %s\n", g_dev, g_bak);
	return r < 0 ? 2 : 1;
}

static int do_restore(const char *path)
{
	FILE *f = fopen(path, "r");
	char magic[8];
	uint64_t off = 0, len = 0;
	uint8_t *buf;

	if (!f || fread(magic, 1, 8, f) != 8 || memcmp(magic, "SDRWBAK1", 8) ||
	    fread(&off, 8, 1, f) != 1 || fread(&len, 8, 1, f) != 1 ||
	    len == 0 || len > MAX_LEN) {
		fprintf(stderr, "%s: not a sd_rwtest backup file\n", path);
		return 3;
	}

	buf = xalloc(len);
	if (fread(buf, 1, len, f) != len) {
		fprintf(stderr, "%s: truncated backup\n", path);
		return 3;
	}

	fclose(f);

	printf("restoring %llu bytes to %s @ 0x%llx\n",
	       (unsigned long long)len, g_dev, (unsigned long long)off);

	struct io_task rst = { .off = (long long)off, .len = len, .data = buf };
	struct io_task vfy = { .off = (long long)off, .len = len, .expect = buf };

	if (run_phase("restore", phase_write, &rst) != 0 ||
	    run_phase("verify", phase_verify, &vfy) != 0) {
		printf("restore: FAILED/HUNG - backup file kept\n");
		return 1;
	}

	printf("restore: done\n");
	return 0;
}

int main(int argc, char **argv)
{
	const char *mode;
	long long size;
	int opt, rc = 0;

	while ((opt = getopt(argc, argv, "d:o:t:b:n:Bh")) != -1) {
		switch (opt) {
		case 'd':
			g_dev = optarg;
			break;

		case 'o':
			g_off = strtoll(optarg, NULL, 0);
			break;

		case 't':
			g_timeout = atoi(optarg);
			break;

		case 'b':
			g_bak = optarg;
			break;

		case 'n':
			g_len = strtoull(optarg, NULL, 0);
			break;

		case 'B':
			g_buffered = 1;
			break;

		default:
			fprintf(stderr, "usage: sd_rwtest [-d dev] [-o off] [-t sec] [-b bakfile] [-n bytes] [-B] [read|write|all|restore <bakfile>]\n");
			return 3;
		}
	}

	if (g_len == 0 || g_len % ALIGN || g_len > MAX_LEN) {
		fprintf(stderr, "-n must be a multiple of %d, at most %d\n",
			ALIGN, MAX_LEN);
		return 3;
	}

	mode = (optind < argc) ? argv[optind] : "all";
	if (strcmp(mode, "restore") == 0) {
		if (optind + 1 >= argc) {
			fprintf(stderr, "restore needs the backup file path\n");
			return 3;
		}

		return do_restore(argv[optind + 1]);
	}

	size = dev_size(g_dev);
	if (size <= 0)
		return 3;

	printf("%s: %lld bytes (%.1f GiB), per-phase timeout %ds\n",
	       g_dev, size, size / (1024.0 * 1024 * 1024), g_timeout);

	if (strcmp(mode, "read") == 0 || strcmp(mode, "all") == 0) {
		int r = test_read();

		if (r > rc)
			rc = r;

		/* a hung read wedges the queue; skip the write test */
		if (r == 2)
			return rc;
	}
	if (strcmp(mode, "write") == 0 || strcmp(mode, "all") == 0) {
		int r = test_write(size);

		if (r > rc)
			rc = r;
	}

	return rc;
}
