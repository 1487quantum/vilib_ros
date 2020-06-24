/*
 * ROS Node wrapper for for CUDA Visual Library by RPG. 
 * config_tracker.hpp
 * Lucas-Kanade Tracker Parameters
 *
 *  __ _  _   ___ ______ ____                    _                   
 * /_ | || | / _ \____  / __ \                  | |                  
 *  | | || || (_) |  / / |  | |_   _  __ _ _ __ | |_ _   _ _ __ ___  
 *  | |__   _> _ <  / /| |  | | | | |/ _` | '_ \| __| | | | '_ ` _ \ 
 *  | |  | || (_) |/ / | |__| | |_| | (_| | | | | |_| |_| | | | | | |
 *  |_|  |_| \___//_/   \___\_\\__,_|\__,_|_| |_|\__|\__,_|_| |_| |_|
 *
 * Copyright (C) 2020 1487Quantum
 * 
 * 
 * Licensed under the MIT License.
 * 
 */

// Frame options
int FRAME_IMAGE_PYRAMID_LEVELS{5};
// Feature detection options
int FEATURE_DETECTOR_CELL_SIZE_WIDTH{32};
int FEATURE_DETECTOR_CELL_SIZE_HEIGHT{32};
int FEATURE_DETECTOR_MIN_LEVEL{0};
int FEATURE_DETECTOR_MAX_LEVEL{5};
int FEATURE_DETECTOR_HORIZONTAL_BORDER{8};
int FEATURE_DETECTOR_VERTICAL_BORDER{8};
// FAST parameters
float FEATURE_DETECTOR_FAST_EPISLON{60.f};
int FEATURE_DETECTOR_FAST_ARC_LENGTH{14};
vilib::fast_score FEATURE_DETECTOR_FAST_SCORE{vilib::SUM_OF_ABS_DIFF_ON_ARC};
//Image publisher
bool enableImgPublish{true};
//Graphics
bool draw_features{true};
int draw_features_thickness{2};
bool draw_features_text{true};
bool draw_bounding_rectangle{true};
int draw_bounding_rectangle_thickness{2};
