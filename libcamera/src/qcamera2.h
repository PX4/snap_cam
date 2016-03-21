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
#ifndef __QCAMERA2_H__
#define __QCAMERA2_H__

#include <hardware/camera.h>
#include <system/camera.h>
#include <vector>

#include "camera.h"

namespace camera
{

enum FrameType
{
    CAMERA_FRAME_PREVIEW,
    CAMERA_FRAME_VIDEO,
    CAMERA_FRAME_PICTURE,
};

class QCamera2Frame : public ICameraFrame
{
private:
    struct camera_device* dev_;
    
    FrameType type;
public:
    QCamera2Frame() : dev_(NULL) {}

    QCamera2Frame(struct camera_device* dev, int64_t timestamp,
                  int32_t msg_type, const camera_memory_t* mem);

    virtual ~QCamera2Frame();

    virtual uint32_t acquireRef();

    virtual uint32_t releaseRef();

    static void dispatchFrame(ICameraListener* listener,
                              struct camera_device* dev, int64_t timestamp,
                              int32_t msg_type, const camera_memory_t* mem);
};

class QCamera2 : public ICameraDevice
{
    struct camera_device* dev_;
    int id_;
    ICameraParameters* params_;
    std::vector<ICameraListener *> listeners_;

    bool isPreviewRequested_;
    bool isPreviewRunning_;
    bool isVideoRunning_;

    static void notify_callback(int32_t msg_type, int32_t ext1, int32_t ext2,
                                void* user);

    static void data_callback(int32_t msg_type, const camera_memory_t* data,
                              unsigned int index,
                              camera_frame_metadata_t* metadata, void* user);

    static void data_timestamp_callback(int64_t timestamp, int32_t msg_type,
                                        const camera_memory_t* data,
                                        unsigned int index, void* user);
public:
    QCamera2();

    virtual ~QCamera2();

    int init(int index);

    int getID() { return id_; }

    /* Implementation of virtual methods of ICameraDevice interface */
    virtual void addListener(ICameraListener* listener);
    virtual void removeListener(ICameraListener* listener);

    virtual void subscribe(uint32_t eventMask);

    virtual void unsubscribe(uint32_t eventMask);

    virtual int setParameters(const ICameraParameters& params);
    virtual int getParameters(uint8_t* buf, uint32_t bufSize,
                              int* bufSizeRequired);

    virtual int takePicture();

    virtual int startPreview();
    virtual void stopPreview();

    virtual int startRecording();
    virtual void stopRecording();

    virtual int startAutoFocus() { return dev_->ops->auto_focus(dev_); }
    virtual void stopAutoFocus() { dev_->ops->cancel_auto_focus(dev_); }

    virtual void cancelPicture() { dev_->ops->cancel_picture(dev_); }
};

} /* namespace camera */

#endif

