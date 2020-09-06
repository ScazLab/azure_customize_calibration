/**
 * The code is adapted from https://github.com/code-iai/iai_kinect2
 */
#include <string>
#include <ros/console.h>

#pragma once
#ifndef __AZURE_CALIBRATION_DEFINITIONS_H__
#define __AZURE_CALIBRATION_DEFINITIONS_H__

#define CALIB_FILE_EXT      ".png"
#define CALIB_FILE_COLOR    "_color" CALIB_FILE_EXT
#define CALIB_FILE_IR       "_ir" CALIB_FILE_EXT
#define CALIB_FILE_IR_GREY  "_grey_ir" CALIB_FILE_EXT
#define CALIB_FILE_DEPTH    "_depth" CALIB_FILE_EXT

#define CALIB_POINTS_COLOR  "_color_points.yaml"
#define CALIB_POINTS_IR     "_ir_points.yaml"

#define CALIB_SYNC          "_sync"
#define CALIB_SYNC_COLOR    CALIB_SYNC CALIB_FILE_COLOR
#define CALIB_SYNC_IR       CALIB_SYNC CALIB_FILE_IR
#define CALIB_SYNC_IR_GREY  CALIB_SYNC CALIB_FILE_IR_GREY

#endif //__AZURE_CALIBRATION_DEFINITIONS_H__
