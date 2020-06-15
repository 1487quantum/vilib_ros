/*
 * ROS Node wrapper for for CUDA Visual Library by RPG. 
 * config_fast.hpp
 * FAST Parameters
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

// Frame preprocessing
int PYRAMID_LEVELS{ 6 };
int PYRAMID_MIN_LEVEL{ 0 };
int PYRAMID_MAX_LEVEL{ PYRAMID_LEVELS };

// FAST detector parameters
float FAST_EPSILON{ 60.0f }; //Threshold level
int FAST_MIN_ARC_LENGTH{ 10 };
// Remark: the Rosten CPU version only works with
//         SUM_OF_ABS_DIFF_ON_ARC and MAX_THRESHOLD
vilib::fast_score FAST_SCORE{ vilib::SUM_OF_ABS_DIFF_ON_ARC };

// NMS parameters
int HORIZONTAL_BORDER{ 0 }; //Horizontal image detection padding (Act as clip off for detection)
int VERTICAL_BORDER{ 0 }; //Vertical image detection padding
int CELL_SIZE_WIDTH{ 16 };
int CELL_SIZE_HEIGHT{ 16 };
