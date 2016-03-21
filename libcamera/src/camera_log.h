/* Copyright (c) 2015, The Linux Foundataion. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of The Linux Foundation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef __CAMERA_LOG_H__
#define __CAMERA_LOG_H__

#include <syslog.h>
#include <stdio.h>

#define CAM_ERR(fmt, args...) do { \
    syslog(LOG_ERR, "%s:%d ERROR: "fmt, __func__, __LINE__, ##args); \
} while (0)

#define CAM_DBG(fmt, args...) do { \
    syslog(LOG_DEBUG, "%s:%d DEBUG: "fmt, __func__, __LINE__, ##args); \
} while (0)

#define CAM_INFO(fmt, args...) do { \
    syslog(LOG_INFO, "%s:%d INFO: "fmt, __func__, __LINE__, ##args); \
} while (0)

#define CAM_PRINT(fmt, args...) do { \
    fprintf(stderr, "%s:%d, "fmt "\n", __func__, __LINE__, ##args); \
    syslog(LOG_INFO, "%s:%d PRINT: "fmt, __func__, __LINE__, ##args); \
} while (0)

#endif
