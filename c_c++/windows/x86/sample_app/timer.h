#include <windows.h>

#define GET_TIME_INIT(num) LARGE_INTEGER _timers[num]; LARGE_INTEGER _freq; QueryPerformanceFrequency(&_freq)

#define GET_TIME_VAL(num) QueryPerformanceCounter(&_timers[num])

#define TIME_VAL_TO_MS(num) ((double)_timers[num].QuadPart*1000.0/_freq.QuadPart)