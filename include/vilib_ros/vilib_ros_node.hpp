/*
 * ROS Node wrapper for for CUDA Visual Library by RPG. 
 * vilib_ros_node.hpp
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

#include <iostream>
#include <vector>
#include <unordered_map>
#include <thread>

#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>

#include "vilib/cuda_common.h"
#include "vilib/preprocess/pyramid.h"
#include "vilib/storage/pyramid_pool.h"
#include "vilib/feature_detection/fast/fast_gpu.h"

#include "vilib_ros/config/config_fast.hpp"
#include "vilib_ros/keypt.h"
#include "vilib_ros/fast_paramConfig.h"

using namespace vilib;

// Pub/Sub
ros::Publisher ptsPub;
image_transport::Publisher imgPub;
image_transport::Subscriber imgSub;

std::shared_ptr<vilib::DetectorBaseGPU> detector_gpu_;

// === FEATURE DETECTOR ===
std::unordered_map<int, int> fDetector(cv_bridge::CvImagePtr imgpt, int image_width_, int image_height_);	//FASt corner detector fx, return all the points detected

// === GRAPHICS ===
void drawText(cv_bridge::CvImagePtr imgpt, int x, int y, std::string msg);						//Draw text
void dCircle(cv_bridge::CvImagePtr imgpt, int x, int y);								//Draw feature point
void dRect(cv_bridge::CvImagePtr imgpt, int x, int y, int w, int h);							//Draw bounding rectangle
void processImg(cv_bridge::CvImagePtr img, std::unordered_map<int, int> pts, int image_width_, int image_height_);	//Draw text & detected features on img

// === DYNAMIC RECONFIG ===
void setDRVals(vilib_ros::fast_paramConfig& config, bool debug);

// === CALLBACK & PUBLISHER ===
void dr_callback(vilib_ros::fast_paramConfig& config, uint32_t level);	//Dynamic reconfigure
void pub_img(cv_bridge::CvImagePtr ipt);				//Publish img with features
void imgCallback(const sensor_msgs::ImageConstPtr& imgp);		//Image Input callback
