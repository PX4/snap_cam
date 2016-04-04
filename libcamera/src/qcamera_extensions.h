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

namespace camera
{

const char KEY_QC_SUPPORTED_HFR_SIZES[] = "hfr-size-values";
const char KEY_QC_PREVIEW_FRAME_RATE_MODE[] = "preview-frame-rate-mode";
const char KEY_QC_SUPPORTED_PREVIEW_FRAME_RATE_MODES[] = "preview-frame-rate-modes";
const char KEY_QC_PREVIEW_FRAME_RATE_AUTO_MODE[] = "frame-rate-auto";
const char KEY_QC_PREVIEW_FRAME_RATE_FIXED_MODE[] = "frame-rate-fixed";
const char KEY_QC_TOUCH_AF_AEC[] = "touch-af-aec";
const char KEY_QC_SUPPORTED_TOUCH_AF_AEC[] = "touch-af-aec-values";
const char KEY_QC_TOUCH_INDEX_AEC[] = "touch-index-aec";
const char KEY_QC_TOUCH_INDEX_AF[] = "touch-index-af";
const char KEY_QC_SCENE_DETECT[] = "scene-detect";
const char KEY_QC_SUPPORTED_SCENE_DETECT[] = "scene-detect-values";
const char KEY_QC_ISO_MODE[] = "iso";
const char KEY_QC_SUPPORTED_ISO_MODES[] = "iso-values";
const char KEY_QC_EXPOSURE_TIME[] = "exposure-time";
const char KEY_QC_MIN_EXPOSURE_TIME[] = "min-exposure-time";
const char KEY_QC_MAX_EXPOSURE_TIME[] = "max-exposure-time";
const char KEY_QC_LENSSHADE[] = "lensshade";
const char KEY_QC_SUPPORTED_LENSSHADE_MODES[] = "lensshade-values";
const char KEY_QC_AUTO_EXPOSURE[] = "auto-exposure";
const char KEY_QC_SUPPORTED_AUTO_EXPOSURE[] = "auto-exposure-values";
const char KEY_QC_DENOISE[] = "denoise";
const char KEY_QC_SUPPORTED_DENOISE[] = "denoise-values";
const char KEY_QC_FOCUS_ALGO[] = "selectable-zone-af";
const char KEY_QC_SUPPORTED_FOCUS_ALGOS[] = "selectable-zone-af-values";
const char KEY_QC_MANUAL_FOCUS_POSITION[] = "manual-focus-position";
const char KEY_QC_MANUAL_FOCUS_POS_TYPE[] = "manual-focus-pos-type";
const char KEY_QC_MIN_FOCUS_POS_INDEX[] = "min-focus-pos-index";
const char KEY_QC_MAX_FOCUS_POS_INDEX[] = "max-focus-pos-index";
const char KEY_QC_MIN_FOCUS_POS_DAC[] = "min-focus-pos-dac";
const char KEY_QC_MAX_FOCUS_POS_DAC[] = "max-focus-pos-dac";
const char KEY_QC_FACE_DETECTION[] = "face-detection";
const char KEY_QC_SUPPORTED_FACE_DETECTION[] = "face-detection-values";
const char KEY_QC_FACE_RECOGNITION[] = "face-recognition";
const char KEY_QC_SUPPORTED_FACE_RECOGNITION[] = "face-recognition-values";
const char KEY_QC_MEMORY_COLOR_ENHANCEMENT[] = "mce";
const char KEY_QC_SUPPORTED_MEM_COLOR_ENHANCE_MODES[] = "mce-values";
const char KEY_QC_DIS[] = "dis";
const char KEY_QC_SUPPORTED_DIS_MODES[] = "dis-values";
const char KEY_QC_VIDEO_HIGH_FRAME_RATE[] = "video-hfr";
const char KEY_QC_VIDEO_HIGH_SPEED_RECORDING[] = "video-hsr";
const char KEY_QC_SUPPORTED_VIDEO_HIGH_FRAME_RATE_MODES[] = "video-hfr-values";
const char KEY_QC_REDEYE_REDUCTION[] = "redeye-reduction";
const char KEY_QC_SUPPORTED_REDEYE_REDUCTION[] = "redeye-reduction-values";
const char KEY_QC_HIGH_DYNAMIC_RANGE_IMAGING[] = "hdr";
const char KEY_QC_SUPPORTED_HDR_IMAGING_MODES[] = "hdr-values";
const char KEY_QC_ZSL[] = "zsl";
const char KEY_QC_SUPPORTED_ZSL_MODES[] = "zsl-values";
const char KEY_QC_ZSL_BURST_INTERVAL[] = "capture-burst-interval";
const char KEY_QC_ZSL_BURST_LOOKBACK[] = "capture-burst-retroactive";
const char KEY_QC_ZSL_QUEUE_DEPTH[] = "capture-burst-queue-depth";
const char KEY_QC_CAMERA_MODE[] = "camera-mode";
const char KEY_QC_AE_BRACKET_HDR[] = "ae-bracket-hdr";
const char KEY_QC_SUPPORTED_AE_BRACKET_MODES[] = "ae-bracket-hdr-values";
const char KEY_QC_SUPPORTED_RAW_FORMATS[] = "raw-format-values";
const char KEY_QC_RAW_FORMAT[] = "raw-format";
const char KEY_QC_ORIENTATION[] = "orientation";
const char KEY_QC_SELECTABLE_ZONE_AF[] = "selectable-zone-af";
const char KEY_QC_CAPTURE_BURST_EXPOSURE[] = "capture-burst-exposures";
const char KEY_QC_NUM_SNAPSHOT_PER_SHUTTER[] = "num-snaps-per-shutter";
const char KEY_QC_NO_DISPLAY_MODE[] = "no-display-mode";
const char KEY_QC_RAW_PICUTRE_SIZE[] = "raw-size";
const char KEY_QC_SUPPORTED_SKIN_TONE_ENHANCEMENT_MODES[] = "skinToneEnhancement-values";
const char KEY_QC_SUPPORTED_LIVESNAPSHOT_SIZES[] = "supported-live-snapshot-sizes";
const char KEY_QC_SCALED_PICTURE_SIZES[] = "scaled-picture-sizes";
const char KEY_QC_HDR_NEED_1X[] = "hdr-need-1x";
const char KEY_QC_PREVIEW_FLIP[] = "preview-flip";
const char KEY_QC_VIDEO_FLIP[] = "video-flip";
const char KEY_QC_SNAPSHOT_PICTURE_FLIP[] = "snapshot-picture-flip";
const char KEY_QC_SUPPORTED_FLIP_MODES[] = "flip-mode-values";
const char KEY_QC_VIDEO_HDR[] = "video-hdr";
const char KEY_QC_VT_ENABLE[] = "avtimer";
const char KEY_QC_SUPPORTED_VIDEO_HDR_MODES[] = "video-hdr-values";
const char KEY_QC_AUTO_HDR_ENABLE [] = "auto-hdr-enable";
const char KEY_QC_SNAPSHOT_BURST_NUM[] = "snapshot-burst-num";
const char KEY_QC_SNAPSHOT_FD_DATA[] = "snapshot-fd-data-enable";
const char KEY_QC_TINTLESS_ENABLE[] = "tintless";
const char KEY_QC_CDS_MODE[] = "cds-mode";
const char KEY_QC_VIDEO_ROTATION[] = "video-rotation";
const char KEY_QC_AF_BRACKET[] = "af-bracket";
const char KEY_QC_SUPPORTED_AF_BRACKET_MODES[] = "af-bracket-values";
const char KEY_QC_CHROMA_FLASH[] = "chroma-flash";
const char KEY_QC_SUPPORTED_CHROMA_FLASH_MODES[] = "chroma-flash-values";
const char KEY_QC_OPTI_ZOOM[] = "opti-zoom";
const char KEY_QC_SUPPORTED_OPTI_ZOOM_MODES[] = "opti-zoom-values";
const char KEY_QC_WB_MANUAL_CCT[] = "wb-manual-cct";
const char KEY_QC_MIN_WB_CCT[] = "min-wb-cct";
const char KEY_QC_MAX_WB_CCT[] = "max-wb-cct";

// Values for effect settings.
const char EFFECT_EMBOSS[] = "emboss";
const char EFFECT_SKETCH[] = "sketch";
const char EFFECT_NEON[] = "neon";

// Values for auto exposure settings.
const char TOUCH_AF_AEC_OFF[] = "touch-off";
const char TOUCH_AF_AEC_ON[] = "touch-on";

// Values for scene mode settings.
const char SCENE_MODE_ASD[] = "asd";   // corresponds to CAMERA_BESTSHOT_AUTO in HAL
const char SCENE_MODE_BACKLIGHT[] = "backlight";
const char SCENE_MODE_FLOWERS[] = "flowers";
const char SCENE_MODE_AR[] = "AR";
const char SCENE_MODE_HDR[] = "hdr";

// Formats for setPreviewFormat and setPictureFormat.
const char PIXEL_FORMAT_YUV420SP_ADRENO[] = "yuv420sp-adreno";
const char PIXEL_FORMAT_YV12[] = "yuv420p";
const char PIXEL_FORMAT_NV12[] = "nv12";
const char QC_PIXEL_FORMAT_NV12_VENUS[] = "nv12-venus";

// Values for raw image formats
const char QC_PIXEL_FORMAT_YUV_RAW_8BIT_YUYV[] = "yuv-raw8-yuyv";
const char QC_PIXEL_FORMAT_YUV_RAW_8BIT_YVYU[] = "yuv-raw8-yvyu";
const char QC_PIXEL_FORMAT_YUV_RAW_8BIT_UYVY[] = "yuv-raw8-uyvy";
const char QC_PIXEL_FORMAT_YUV_RAW_8BIT_VYUY[] = "yuv-raw8-vyuy";
const char QC_PIXEL_FORMAT_BAYER_QCOM_RAW_8GBRG[] = "bayer-qcom-8gbrg";
const char QC_PIXEL_FORMAT_BAYER_QCOM_RAW_8GRBG[] = "bayer-qcom-8grbg";
const char QC_PIXEL_FORMAT_BAYER_QCOM_RAW_8RGGB[] = "bayer-qcom-8rggb";
const char QC_PIXEL_FORMAT_BAYER_QCOM_RAW_8BGGR[] = "bayer-qcom-8bggr";
const char QC_PIXEL_FORMAT_BAYER_QCOM_RAW_10GBRG[] = "bayer-qcom-10gbrg";
const char QC_PIXEL_FORMAT_BAYER_QCOM_RAW_10GRBG[] = "bayer-qcom-10grbg";
const char QC_PIXEL_FORMAT_BAYER_QCOM_RAW_10RGGB[] = "bayer-qcom-10rggb";
const char QC_PIXEL_FORMAT_BAYER_QCOM_RAW_10BGGR[] = "bayer-qcom-10bggr";
const char QC_PIXEL_FORMAT_BAYER_QCOM_RAW_12GBRG[] = "bayer-qcom-12gbrg";
const char QC_PIXEL_FORMAT_BAYER_QCOM_RAW_12GRBG[] = "bayer-qcom-12grbg";
const char QC_PIXEL_FORMAT_BAYER_QCOM_RAW_12RGGB[] = "bayer-qcom-12rggb";
const char QC_PIXEL_FORMAT_BAYER_QCOM_RAW_12BGGR[] = "bayer-qcom-12bggr";
const char QC_PIXEL_FORMAT_BAYER_MIPI_RAW_8GBRG[] = "bayer-mipi-8gbrg";
const char QC_PIXEL_FORMAT_BAYER_MIPI_RAW_8GRBG[] = "bayer-mipi-8grbg";
const char QC_PIXEL_FORMAT_BAYER_MIPI_RAW_8RGGB[] = "bayer-mipi-8rggb";
const char QC_PIXEL_FORMAT_BAYER_MIPI_RAW_8BGGR[] = "bayer-mipi-8bggr";
const char QC_PIXEL_FORMAT_BAYER_MIPI_RAW_10GBRG[] = "bayer-mipi-10gbrg";
const char QC_PIXEL_FORMAT_BAYER_MIPI_RAW_10GRBG[] = "bayer-mipi-10grbg";
const char QC_PIXEL_FORMAT_BAYER_MIPI_RAW_10RGGB[] = "bayer-mipi-10rggb";
const char QC_PIXEL_FORMAT_BAYER_MIPI_RAW_10BGGR[] = "bayer-mipi-10bggr";
const char QC_PIXEL_FORMAT_BAYER_MIPI_RAW_12GBRG[] = "bayer-mipi-12gbrg";
const char QC_PIXEL_FORMAT_BAYER_MIPI_RAW_12GRBG[] = "bayer-mipi-12grbg";
const char QC_PIXEL_FORMAT_BAYER_MIPI_RAW_12RGGB[] = "bayer-mipi-12rggb";
const char QC_PIXEL_FORMAT_BAYER_MIPI_RAW_12BGGR[] = "bayer-mipi-12bggr";
const char QC_PIXEL_FORMAT_BAYER_IDEAL_QCOM_8GBRG[] = "bayer-ideal-qcom-8gbrg";
const char QC_PIXEL_FORMAT_BAYER_IDEAL_QCOM_8GRBG[] = "bayer-ideal-qcom-8grbg";
const char QC_PIXEL_FORMAT_BAYER_IDEAL_QCOM_8RGGB[] = "bayer-ideal-qcom-8rggb";
const char QC_PIXEL_FORMAT_BAYER_IDEAL_QCOM_8BGGR[] = "bayer-ideal-qcom-8bggr";
const char QC_PIXEL_FORMAT_BAYER_IDEAL_QCOM_10GBRG[] = "bayer-ideal-qcom-10gbrg";
const char QC_PIXEL_FORMAT_BAYER_IDEAL_QCOM_10GRBG[] = "bayer-ideal-qcom-10grbg";
const char QC_PIXEL_FORMAT_BAYER_IDEAL_QCOM_10RGGB[] = "bayer-ideal-qcom-10rggb";
const char QC_PIXEL_FORMAT_BAYER_IDEAL_QCOM_10BGGR[] = "bayer-ideal-qcom-10bggr";
const char QC_PIXEL_FORMAT_BAYER_IDEAL_QCOM_12GBRG[] = "bayer-ideal-qcom-12gbrg";
const char QC_PIXEL_FORMAT_BAYER_IDEAL_QCOM_12GRBG[] = "bayer-ideal-qcom-12grbg";
const char QC_PIXEL_FORMAT_BAYER_IDEAL_QCOM_12RGGB[] = "bayer-ideal-qcom-12rggb";
const char QC_PIXEL_FORMAT_BAYER_IDEAL_QCOM_12BGGR[] = "bayer-ideal-qcom-12bggr";
const char QC_PIXEL_FORMAT_BAYER_IDEAL_MIPI_8GBRG[] = "bayer-ideal-mipi-8gbrg";
const char QC_PIXEL_FORMAT_BAYER_IDEAL_MIPI_8GRBG[] = "bayer-ideal-mipi-8grbg";
const char QC_PIXEL_FORMAT_BAYER_IDEAL_MIPI_8RGGB[] = "bayer-ideal-mipi-8rggb";
const char QC_PIXEL_FORMAT_BAYER_IDEAL_MIPI_8BGGR[] = "bayer-ideal-mipi-8bggr";
const char QC_PIXEL_FORMAT_BAYER_IDEAL_MIPI_10GBRG[] = "bayer-ideal-mipi-10gbrg";
const char QC_PIXEL_FORMAT_BAYER_IDEAL_MIPI_10GRBG[] = "bayer-ideal-mipi-10grbg";
const char QC_PIXEL_FORMAT_BAYER_IDEAL_MIPI_10RGGB[] = "bayer-ideal-mipi-10rggb";
const char QC_PIXEL_FORMAT_BAYER_IDEAL_MIPI_10BGGR[] = "bayer-ideal-mipi-10bggr";
const char QC_PIXEL_FORMAT_BAYER_IDEAL_MIPI_12GBRG[] = "bayer-ideal-mipi-12gbrg";
const char QC_PIXEL_FORMAT_BAYER_IDEAL_MIPI_12GRBG[] = "bayer-ideal-mipi-12grbg";
const char QC_PIXEL_FORMAT_BAYER_IDEAL_MIPI_12RGGB[] = "bayer-ideal-mipi-12rggb";
const char QC_PIXEL_FORMAT_BAYER_IDEAL_MIPI_12BGGR[] = "bayer-ideal-mipi-12bggr";
const char QC_PIXEL_FORMAT_BAYER_IDEAL_PLAIN8_8GBRG[] = "bayer-ideal-plain8-8gbrg";
const char QC_PIXEL_FORMAT_BAYER_IDEAL_PLAIN8_8GRBG[] = "bayer-ideal-plain8-8grbg";
const char QC_PIXEL_FORMAT_BAYER_IDEAL_PLAIN8_8RGGB[] = "bayer-ideal-plain8-8rggb";
const char QC_PIXEL_FORMAT_BAYER_IDEAL_PLAIN8_8BGGR[] = "bayer-ideal-plain8-8bggr";
const char QC_PIXEL_FORMAT_BAYER_IDEAL_PLAIN16_8GBRG[] = "bayer-ideal-plain16-8gbrg";
const char QC_PIXEL_FORMAT_BAYER_IDEAL_PLAIN16_8GRBG[] = "bayer-ideal-plain16-8grbg";
const char QC_PIXEL_FORMAT_BAYER_IDEAL_PLAIN16_8RGGB[] = "bayer-ideal-plain16-8rggb";
const char QC_PIXEL_FORMAT_BAYER_IDEAL_PLAIN16_8BGGR[] = "bayer-ideal-plain16-8bggr";
const char QC_PIXEL_FORMAT_BAYER_IDEAL_PLAIN16_10GBRG[] = "bayer-ideal-plain16-10gbrg";
const char QC_PIXEL_FORMAT_BAYER_IDEAL_PLAIN16_10GRBG[] = "bayer-ideal-plain16-10grbg";
const char QC_PIXEL_FORMAT_BAYER_IDEAL_PLAIN16_10RGGB[] = "bayer-ideal-plain16-10rggb";
const char QC_PIXEL_FORMAT_BAYER_IDEAL_PLAIN16_10BGGR[] = "bayer-ideal-plain16-10bggr";
const char QC_PIXEL_FORMAT_BAYER_IDEAL_PLAIN16_12GBRG[] = "bayer-ideal-plain16-12gbrg";
const char QC_PIXEL_FORMAT_BAYER_IDEAL_PLAIN16_12GRBG[] = "bayer-ideal-plain16-12grbg";
const char QC_PIXEL_FORMAT_BAYER_IDEAL_PLAIN16_12RGGB[] = "bayer-ideal-plain16-12rggb";
const char QC_PIXEL_FORMAT_BAYER_IDEAL_PLAIN16_12BGGR[] = "bayer-ideal-plain16-12bggr";

// Values for auto exposure settings.
const char AUTO_EXPOSURE_FRAME_AVG[] = "frame-average";
const char AUTO_EXPOSURE_CENTER_WEIGHTED[] = "center-weighted";
const char AUTO_EXPOSURE_SPOT_METERING[] = "spot-metering";
const char AUTO_EXPOSURE_SMART_METERING[] = "smart-metering";
const char AUTO_EXPOSURE_USER_METERING[] = "user-metering";
const char AUTO_EXPOSURE_SPOT_METERING_ADV[] = "spot-metering-adv";
const char AUTO_EXPOSURE_CENTER_WEIGHTED_ADV[] = "center-weighted-adv";

const char KEY_QC_GPS_LATITUDE_REF[] = "gps-latitude-ref";
const char KEY_QC_GPS_LONGITUDE_REF[] = "gps-longitude-ref";
const char KEY_QC_GPS_ALTITUDE_REF[] = "gps-altitude-ref";
const char KEY_QC_GPS_STATUS[] = "gps-status";

const char KEY_QC_HISTOGRAM[] = "histogram";
const char KEY_QC_SUPPORTED_HISTOGRAM_MODES[] = "histogram-values";

const char VALUE_ENABLE[] = "enable";
const char VALUE_DISABLE[] = "disable";
const char VALUE_OFF[] = "off";
const char VALUE_ON[] = "on";
const char VALUE_TRUE[] = "true";
const char VALUE_FALSE[] = "false";

const char KEY_QC_SHARPNESS[] = "sharpness";
const char KEY_QC_MIN_SHARPNESS[] = "min-sharpness";
const char KEY_QC_MAX_SHARPNESS[] = "max-sharpness";
const char KEY_QC_SHARPNESS_STEP[] = "sharpness-step";
const char KEY_QC_CONTRAST[] = "contrast";
const char KEY_QC_MIN_CONTRAST[] = "min-contrast";
const char KEY_QC_MAX_CONTRAST[] = "max-contrast";
const char KEY_QC_CONTRAST_STEP[] = "contrast-step";
const char KEY_QC_SATURATION[] = "saturation";
const char KEY_QC_MIN_SATURATION[] = "min-saturation";
const char KEY_QC_MAX_SATURATION[] = "max-saturation";
const char KEY_QC_SATURATION_STEP[] = "saturation-step";
const char KEY_QC_BRIGHTNESS[] = "luma-adaptation";
const char KEY_QC_MIN_BRIGHTNESS[] = "min-brightness";
const char KEY_QC_MAX_BRIGHTNESS[] = "max-brightness";
const char KEY_QC_BRIGHTNESS_STEP[] = "brightness-step";
const char KEY_QC_SCE_FACTOR[] = "skinToneEnhancement";
const char KEY_QC_MIN_SCE_FACTOR[] = "min-sce-factor";
const char KEY_QC_MAX_SCE_FACTOR[] = "max-sce-factor";
const char KEY_QC_SCE_FACTOR_STEP[] = "sce-factor-step";

const char KEY_QC_SUPPORTED_CAMERA_FEATURES[] = "qc-camera-features";
const char KEY_QC_MAX_NUM_REQUESTED_FACES[] = "qc-max-num-requested-faces";

//Values for DENOISE
const char DENOISE_OFF[] = "denoise-off";
const char DENOISE_ON[] = "denoise-on";

// Values for selectable zone af Settings
const char FOCUS_ALGO_AUTO[] = "auto";
const char FOCUS_ALGO_SPOT_METERING[] = "spot-metering";
const char FOCUS_ALGO_CENTER_WEIGHTED[] = "center-weighted";
const char FOCUS_ALGO_FRAME_AVERAGE[] = "frame-average";

// Values for HFR settings.
const char VIDEO_HFR_OFF[] = "off";
const char VIDEO_HFR_2X[] = "60";
const char VIDEO_HFR_3X[] = "90";
const char VIDEO_HFR_4X[] = "120";
const char VIDEO_HFR_5X[] = "150";

// Values for HDR Bracketing settings.
const char AE_BRACKET_OFF[] = "Off";
const char AE_BRACKET[] = "AE-Bracket";

// Values for AF Bracketing setting.
const char AF_BRACKET_OFF[] = "af-bracket-off";
const char AF_BRACKET_ON[] = "af-bracket-on";

// Values for Chroma Flash setting.
const char CHROMA_FLASH_OFF[] = "chroma-flash-off";
const char CHROMA_FLASH_ON[] = "chroma-flash-on";

// Values for Opti Zoom setting.
const char OPTI_ZOOM_OFF[] = "opti-zoom-off";
const char OPTI_ZOOM_ON[] = "opti-zoom-on";

// Values for FLIP settings.
const char FLIP_MODE_OFF[] = "off";
const char FLIP_MODE_V[] = "flip-v";
const char FLIP_MODE_H[] = "flip-h";
const char FLIP_MODE_VH[] = "flip-vh";

const char CDS_MODE_OFF[] = "off";
const char CDS_MODE_ON[] = "on";
const char CDS_MODE_AUTO[] = "auto";

const char KEY_SELECTED_AUTO_SCENE[] = "selected-auto-scene";

const char KEY_QC_EXPOSURE_MANUAL[] = "qc-exposure-manual";
const char KEY_QC_GAIN_MANUAL[] = "qc-gain-manual";
const char KEY_QC_VERTICAL_FLIP[] = "qc-vertical-flip";
const char KEY_QC_HORIZONTAL_MIRROR[] = "qc-horizontal-mirror";


} /* namespace camera */
