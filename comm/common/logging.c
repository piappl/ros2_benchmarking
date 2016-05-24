#include <fcntl.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <time.h>
#include <common/logging.h>

int debug_level = LOG_BENCHMARK;

static char *too_long = "***** Log string is too long *****\n";

static void write_log(const void *data, unsigned size)
{
	write(STDERR_FILENO, data, size); // ignore write errors, could be closed descriptor
}

int debug_open(const char *file_name)
{
	int newfd = open(file_name, O_APPEND | O_WRONLY | O_CREAT, S_IRUSR | S_IWUSR);
	if (newfd < 0) {
		debug(LOG_ERROR, "System", "Unable to open log file %s: %m\n", file_name);
		return -1;
	}

	int err = dup2(newfd, STDERR_FILENO);
	close(newfd);

	if (err < 0) {
		debug(LOG_ERROR, "System", "Unable to set log file %s as stderr: %m\n", file_name);
		return -1;
	}

	return 0;
}

void debug(unsigned level, char const *facility, const char *format, ...)
{
	if (level > (unsigned)debug_level)
		return;

    char *timestamp = "";
    char tmbuf[64], buf[64];
    if (level == LOG_BENCHMARK)
    {
        struct timeval tv;
        time_t nowtime;
        struct tm *nowtm;
        gettimeofday(&tv, NULL);
        nowtime = tv.tv_sec;
        nowtm = localtime(&nowtime);
        strftime(tmbuf, sizeof tmbuf, "%Y-%m-%d %H:%M:%S", nowtm);
        snprintf(buf, sizeof buf, "%s.%06ld", tmbuf, tv.tv_usec);
        timestamp = buf;
    }

	char buffer[1024];    
    int len = snprintf(buffer, sizeof(buffer), "%s %s%s: ", timestamp, level == LOG_ERROR ? "ERROR! " : level == LOG_WARNING ? "WARNING! " : level == LOG_BENCHMARK ? "BENCHMARK! " : "", facility);

	if (len < 0)
		return;					// what can we do?

	if (len < (int)sizeof(buffer)) {
		va_list ap;
		va_start(ap, format);
		int len2 = vsnprintf(buffer + len, sizeof(buffer) - len, format, ap);
		if (len < 0)
			return;				// what can we do?
		len += len2;
		va_end(ap);
	}

	if ((unsigned)len < sizeof(buffer)) {
		buffer[len++] = '\n';
		write_log(buffer, len);
	} else
		write_log(too_long, strlen(too_long));
}

// expects 3 * len bytes of memory at 'buffer'
void dump_to_string(char *buffer, unsigned char *data, unsigned len)
{
	while (len--) {
		sprintf(buffer, "%02X%s", *data++, len == 0 ? "" : " ");
		buffer += 3;
	}
}
