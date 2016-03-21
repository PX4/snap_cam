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
#include <stdint.h>
#include <hardware/camera.h>
#include "qcamera2.h"
#include <QComOMXMetadata.h>

namespace camera
{

enum MemType
{
    MEM_MAPPED,
    MEM_ALLOCATED,
    MEM_INVALID,
};

const int NH_NUM_FDS = 1;
const int NH_NUM_INTS = 2;

class CameraMemory
{

 public:
    /* create a new memory object and map/allocate the buffer */
    CameraMemory(int fd, uint32_t size);

    /* unmap/free the buffer and destroy the memory object */
    ~CameraMemory();

    /* function to serve the request_memory_callback from camera HAL */
    static camera_memory_t* requestMemory(int fd, size_t buf_size,
                                          unsigned int num_bufs,
                                          void* user);
    /* function to serve the release_memory_callback from camera HAL */
    static void releaseMemory(struct camera_memory* mem);

    QCamera2Frame frame;
 private:
    bool valid_;
    android::encoder_media_buffer_type metadata_;
    /* pre-allocated memory for native_handle_t with data */
    uint8_t nh_mem_[sizeof(native_handle_t) +
        (NH_NUM_FDS + NH_NUM_INTS) * sizeof(int)];
    camera_memory_t* mem_;
    MemType type_;
};

} /* namespace camera */
