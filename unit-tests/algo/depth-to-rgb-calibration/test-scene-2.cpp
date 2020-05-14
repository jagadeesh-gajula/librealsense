// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Intel Corporation. All Rights Reserved.

//#cmake:add-file ../../../src/algo/depth-to-rgb-calibration/*.cpp

#include "d2rgb-common.h"
#include "compare-to-bin-file.h"
#include "compare-scene.h"


TEST_CASE("Scene 2", "[d2rgb]")
{
    std::string scene_dir("C:\\Users\\nyassin\\Documents\\realsense_all\\debug_scene");// "..\\unit-tests\\algo\\depth-to-rgb-calibration\\19.2.20" );
//    std::string scene_dir( "C:\\work\\autocal" );
    scene_dir += "\\LongRange 768X1024 (RGB 1920X1080)\\2\\";// "\\F9440687\\LongRange_D_768x1024_RGB_1920x1080\\2\\";

    compare_scene( scene_dir );
}