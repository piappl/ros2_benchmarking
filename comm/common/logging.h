#ifndef TASKCORE_LOGGING_H
#define TASKCORE_LOGGING_H

#define LOG_ERROR   0
#define LOG_WARNING 1
#define LOG_BENCHMARK 2
#define LOG_DEBUG   3

#ifdef __cplusplus
extern "C" {
#endif

extern int debug_level;

int debug_open(const char *file_name);
void debug(unsigned level, const char *facility, const char *format, ...) __attribute__((format(printf, 3, 4)));
// expects 3 * len bytes of memory at 'buffer'
void dump_to_string(char *buffer, unsigned char *data, unsigned len);

#ifdef __cplusplus
}
#endif

#endif //TASKCORE_LOGGING_H
