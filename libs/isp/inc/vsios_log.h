/****************************************************************************
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2014-2024 Vivante Corporation.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 *****************************************************************************/

#ifndef __VSIOS_LOG_H__
#define __VSIOS_LOG_H__

#ifdef __cplusplus
#if __cplusplus
extern "C"{
#endif
#endif

#include <stdio.h>
#include <stdlib.h>

#define LOG_COLOR_RED    "\033[1;30;31m"
#define LOG_COLOR_YELLOW "\033[1;30;33m"
#define LOG_COLOR_GREEN  "\033[1;30;32m"
#define LOG_COLOR_WHITE  "\033[1;30;37m"
#define LOG_COLOR_RESET  "\033[0m"

#define __ALOG_INT(format, ...)                                     \
            "[%s] " format LOG_COLOR_RESET "%s", LOGTAG, __VA_ARGS__

typedef enum VsiLogLevel_e {
    VSI_LOG_LEVEL_NONE = 0, //No debug information is output.
    VSI_LOG_LEVEL_ERR ,     //Logs all fatal errors.
    VSI_LOG_LEVEL_WARN,     //Logs all warnings.
    VSI_LOG_LEVEL_INFO,     //Logs all informational messages.
    VSI_LOG_LEVEL_DEBUG,    //Logs all debug messages.
    VSI_LOG_LEVEL_VERBOSE,
} VsiLogLevel_t;

#define VSI_PRINT printf


#define VSI_ALOGE(...)                                             \
    if (VsiLogLevel() >= VSI_LOG_LEVEL_ERR && VsiLogPrint)                        \
        VsiLogPrint(LOG_COLOR_RED "ERROR:" __ALOG_INT(__VA_ARGS__, ""))

#define VSI_ALOGW(...)                                             \
    if (VsiLogLevel() >= VSI_LOG_LEVEL_WARN && VsiLogPrint)                       \
        VsiLogPrint(LOG_COLOR_YELLOW "WARN:" __ALOG_INT(__VA_ARGS__, ""))

#define VSI_ALOGI(...)                                             \
    if (VsiLogLevel() >= VSI_LOG_LEVEL_INFO && VsiLogPrint)                       \
        VsiLogPrint(LOG_COLOR_GREEN "INFO:" __ALOG_INT(__VA_ARGS__, ""))

#define VSI_ALOGD(...)                                             \
    if (VsiLogLevel() >= VSI_LOG_LEVEL_DEBUG && VsiLogPrint)                      \
        VsiLogPrint(LOG_COLOR_WHITE "DEBUG:" __ALOG_INT(__VA_ARGS__, ""))


extern int (*VsiLogLevel)(void);
extern int (*VsiLogPrint)(const char *fmt, ...);
void VsiLogLevelSet(int (*pfnLogLevel)(void), int (*pfnLogPrint)(const char *fmt, ...));

extern int (*VsiVbBufCount)(void);
void VsiVbBufCountSet(int (*pfnBufCount)(void));

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif

#endif
