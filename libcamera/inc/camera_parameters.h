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
#ifndef __CAMERA_PARAMETERS_H__
#define __CAMERA_PARAMETERS_H__

#include <vector>
#include <string>

#include "camera.h"

namespace camera
{

/**
 * Image frame dimensions
 */
struct ImageSize
{
    int width;  /*!< Image width in pixels */
    int height; /*!< Image height in pixels */

    ImageSize() : width(0), height(0) {}
    ImageSize(int w, int h) : width(w), height(h) {}
};

/**
 * Structure for storing values for ranged parameters such as
 * brightness, contrast, fps etc.
 */
struct Range
{
    int min;    /*!< minimum value */
    int max;    /*!< maximum value */
    int step;   /*!< step increment for intermediate values */

    Range() : min(0), max(0), step(0) {}
    Range(int m, int x, int s) : min(m), max(x), step(s) {}
};

/**
 * Available values for video FPS
 */
enum VideoFPS
{
    VIDEO_FPS_30 = 30,   /*!< 30fps regular framerate mode */
    VIDEO_FPS_60 = 60,   /*!< 60fps High framerate mode */
    VIDEO_FPS_90 = 90,   /*!< 90fps High framerate mode */
    VIDEO_FPS_120 = 120, /*!< 120fps High framerate mode */
    VIDEO_FPS_150 = 150, /*!< 150fps High framerate mode */
};

/* focus mode values */
const std::string FOCUS_MODE_AUTO = "auto";
const std::string FOCUS_MODE_INFINITY = "infinity";
const std::string FOCUS_MODE_MACRO = "macro";
const std::string FOCUS_MODE_FIXED = "fixed";
const std::string FOCUS_MODE_EDOF = "edof";
const std::string FOCUS_MODE_CONTINUOUS_VIDEO = "continuous-video";
const std::string FOCUS_MODE_CONTINUOUS_PICTURE = "continuous-picture";
const std::string FOCUS_MODE_MANUAL_POSITION = "manual";

/* ISO values */
const std::string ISO_AUTO = "auto";
const std::string ISO_HJR = "ISO_HJR";
const std::string ISO_100 = "ISO100";
const std::string ISO_200 = "ISO200";
const std::string ISO_400 = "ISO400";
const std::string ISO_800 = "ISO800";
const std::string ISO_1600 = "ISO1600";
const std::string ISO_3200 = "ISO3200";

/* White Balance values */
const std::string WHITE_BALANCE_AUTO = "auto";
const std::string WHITE_BALANCE_INCANDESCENT = "incandescent";
const std::string WHITE_BALANCE_FLUORESCENT = "fluorescent";
const std::string WHITE_BALANCE_WARM_FLUORESCENT = "warm-fluorescent";
const std::string WHITE_BALANCE_DAYLIGHT = "daylight";
const std::string WHITE_BALANCE_CLOUDY_DAYLIGHT = "cloudy-daylight";
const std::string WHITE_BALANCE_TWILIGHT = "twilight";
const std::string WHITE_BALANCE_SHADE = "shade";
const std::string WHITE_BALANCE_MANUAL_CCT = "manual-cct";

class CameraParams : public ICameraParameters
{
public:

    CameraParams();

    virtual ~CameraParams();

    /**
     * initialize the object by getting current state of parameters
     * from device.
     *
     * @param device : A valid camera device object
     *
     * @return int : 0 on success
     */
    int init(ICameraDevice* device);

    virtual int writeObject(std::ostream& ps) const;

    /**
     * get a string representation of the object
     *
     * @return std::string
     */
    std::string toString() const;

    /**
     * Updates the current state of the parameters to camera device.
     * Fails for any invalid entries.
     *
     * @return int : 0 on success
     */
    int commit();

    /**
     * get preview sizes supported by the camera
     *
     * @return std::vector<ImageSize> : list of preview sizes
     */
    std::vector<ImageSize> getSupportedPreviewSizes() const;

    /**
     * set preview size
     *
     * @param size
     */
    void setPreviewSize(const ImageSize& size);

    /**
     * get current preview size
     *
     * @return ImageSize
     */
    ImageSize getPreviewSize() const;

    /**
     * get video sizes supported by the camera
     *
     * @return std::vector<ImageSize> : list of video sizes
     */
    std::vector<ImageSize> getSupportedVideoSizes() const;

    /**
     * get current video size
     *
     * @return ImageSize
     */
    ImageSize getVideoSize() const;

    /**
     * set video size
     *
     * @param size
     */
    void setVideoSize(const ImageSize& size);

    /**
     * get picture sizes supported by the camera
     *
     * @return std::vector<ImageSize> : list of picture sizes
     */
    std::vector<ImageSize> getSupportedPictureSizes() const;

    /**
     * get current picture size
     *
     * @return ImageSize
     */
    ImageSize getPictureSize() const;

    /**
     * set picture size
     *
     * see @ref takePicture
     *
     * @param size
     */
    void setPictureSize(const ImageSize& size);


    /**
     * generic get function to get string representation of value of
     * a parameter using a key.
     *
     * @param key [in]
     *
     * @return std::string : value
     */
    virtual std::string get(const std::string& key) const;

    /**
     * generic set function to set value of a parameter using
     * key-value pair
     *
     * @param key [in]
     * @param value [in]
     */
    virtual void set(const std::string& key, const std::string& value);

    /**
     * get a list of supported focus modes
     *
     * @return vector<string> : focus mode values
     */
    std::vector<std::string> getSupportedFocusModes() const;

    /**
     * get current value of focus mode
     *
     * @return std::string
     */
    std::string getFocusMode() const;

    /**
     * set focus mode value
     *
     * @param value
     */
    void setFocusMode(const std::string& value);

    /**
     * get a list of supported whitebalance modes
     *
     * @return vector<string> : whitebalance values
     */
    std::vector<std::string> getSupportedWhiteBalance() const;

    /**
     * get current value of whitebalance mode
     *
     * @return std::string
     */
    std::string getWhiteBalance() const;

    /**
     * set whitebalance mode value
     *
     * @param value
     */
    void setWhiteBalance(const std::string& value);

    /**
     * get a list of supported ISO modes
     *
     * @return vector<string> : ISO values
     */
    std::vector<std::string> getSupportedISO() const;

    /**
     * get current value of ISO mode
     *
     * @return std::string
     */
    std::string getISO() const;

    /**
     * set ISO mode value
     *
     * @param value
     */
    void setISO(const std::string& value);

    /**
     * get a range of supported sharpness values
     *
     * @return Range : sharpness range
     */
    Range getSupportedSharpness() const;

    /**
     * get current sharpness value
     *
     * @return int
     */
    int getSharpness() const;

    /**
     * set sharpness value
     *
     * @param value
     */
    void setSharpness(int value);

    /**
     * get a range of supported brightness values
     *
     * @return Range : brightness range
     */
    Range getSupportedBrightness() const;

    /**
     * get current brightness value
     *
     * @return int
     */
    int getBrightness() const;

    /**
     * set brightness value
     *
     * @param value
     */
    void setBrightness(int value);

    /**
     * get a range of supported contrast values
     *
     * @return Range : contrast range
     */
    Range getSupportedContrast() const;

    /**
     * get current contrast value
     *
     * @return int
     */
    int getContrast() const;

    /**
     * set contrast value
     *
     * @param value
     */
    void setContrast(int value);

    /**
     * get supported ranges for preview FPS The FPS range has valid
     * min and max value. Actual fixed point FPS value is calculated
     * by dividing the min and max values by 1000. For example, max
     * value of 26123 represents 26.123 fps.
     *
     * @return vector<Range> : preview fps ranges
     */
    std::vector<Range> getSupportedPreviewFpsRanges() const;

    /**
     * get current preview fps range value
     *
     * @return Range
     */
    Range getPreviewFpsRange() const;

    /**
     * set preview fps range value
     *
     * @param value
     */
    void setPreviewFpsRange(const Range& value);

    /**
     * get a list of fixed FPS values for video stream.
     *
     * @return vector<VideoFPS> : video fps values
     */
    std::vector<VideoFPS> getSupportedVideoFps() const;

    /**
     * get current video fps mode
     *
     * @return VideoFPS
     */
    VideoFPS getVideoFPS() const;

    /**
     * set video fps mode.
     *
     * Note: Setting the mode to high framerate will override
     * preview FPS settings. see \ref VideoFPS for high framerate
     * values.
     *
     * @param VideoFPS
     */
    void setVideoFPS(VideoFPS value);

    /**
     * get a list of supported preview formats
     *
     * @return std::vector<std::string>
     */
    std::vector<std::string> getSupportedPreviewFormats() const;

    /**
     * get current preview format
     *
     * @return std::string
     */
    std::string getPreviewFormat() const;

    /**
     * set preview format
     *
     * @param value
     */
    void setPreviewFormat(const std::string& value);

     /**
     * set manual exposure in sensor
     *
     * @param value
     */
    void setManualExposure(int value);

    /**
     * set manual gain in sensor
     *
     * @param value
     */
    void setManualGain(int value);

    /**
     * set vertical flip bit in sensor
     *
     * @param value
     */
    void setVerticalFlip(bool value);

    /**
     * set horizontal mirror bit in sensor
     *
     * @param value
     */
    void setHorizontalMirror(bool value);

private:

    /**
     * private implementation and storage handle for parameters
     */
    void *priv_;

    /**
     * handle to attached camera device
     */
    ICameraDevice *device_;
}; /* class CameraParams */

} /* namespace camera */

#endif  /* __CAMERA_PARAMETERS_H__ */

