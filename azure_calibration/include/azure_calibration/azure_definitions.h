/**
 * The code is adapted from https://github.com/code-iai/iai_kinect2
 */

#pragma once
#ifndef __AZURE_DEFINITIONS_H__
#define __AZURE_DEFINITIONS_H__

#include <azure_calibration/azure_console.h>

#define AZURE_DEFAULT_NS          ""

#define AZURE_TF_LINK             "_link"
#define AZURE_TF_RGB_OPT_FRAME    "rgb_camera_frame"
#define AZURE_TF_IR_OPT_FRAME     "depth_camera_frame"


#define AZURE_TOPIC_IMAGE_RECT    "_rect"
#define AZURE_TOPIC_IMAGE_COLOR   "/rgb/image_raw"
#define AZURE_TOPIC_IMAGE_MONO    "/rgb/image_raw"
#define AZURE_TOPIC_IMAGE_DEPTH   "/depth/image_raw"
#define AZURE_TOPIC_IMAGE_IR      "/ir/image_raw"

#define AZURE_TOPIC_COMPRESSED    "/compressed"
#define AZURE_TOPIC_INFO          "/camera_info"

#define AZURE_CALIB_COLOR         "calib_color.yaml"
#define AZURE_CALIB_IR            "calib_ir.yaml"
#define AZURE_CALIB_POSE          "calib_pose.yaml"
#define AZURE_CALIB_DEPTH         "calib_depth.yaml"

#define AZURE_CALIB_CAMERA_MATRIX "cameraMatrix"
#define AZURE_CALIB_DISTORTION    "distortionCoefficients"
#define AZURE_CALIB_ROTATION      "rotation"
#define AZURE_CALIB_PROJECTION    "projection"
#define AZURE_CALIB_TRANSLATION   "translation"
#define AZURE_CALIB_ESSENTIAL     "essential"
#define AZURE_CALIB_FUNDAMENTAL   "fundamental"
#define AZURE_CALIB_DEPTH_SHIFT   "depthShift"

#endif //__AZURE_DEFINITIONS_H__
