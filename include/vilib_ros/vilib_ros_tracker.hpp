/*
 * ROS Node wrapper for for CUDA Visual Library by RPG. 
 * vilib_ros_tracker.hpp
 * Feature Tracker Node Header
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

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/cudev/common.hpp>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>

#include "vilib/cuda_common.h"
#include "vilib/preprocess/pyramid.h"
#include "vilib/storage/pyramid_pool.h"
#include "vilib/feature_detection/fast/fast_gpu.h"
#include "vilib/feature_tracker/feature_tracker_gpu.h"

#include "vilib_ros/config/config_tracker.hpp"
#include "vilib_ros/keypt.h"
#include "vilib_ros/tracker_paramConfig.h"

using namespace vilib;

// Pub/Sub
ros::Publisher ptsPub;
image_transport::Publisher imgPub;
image_transport::Subscriber imgSub;

// === FEATURE TRACKER ===
std::shared_ptr<vilib::DetectorBaseGPU> detector_gpu_;
std::shared_ptr<vilib::FeatureTrackerBase> tracker_gpu_;

//Tracker
bool initialized_{ false };
std::size_t total_tracked_ftr_cnt, total_detected_ftr_cnt;
std::shared_ptr<vilib::Frame> iTracker(cv_bridge::CvImagePtr imgpt, int image_width_, int image_height_);	//Feature Tracker fx, return all the points detected

// === GRAPHICS ===
void drawText(cv_bridge::CvImagePtr imgpt, int x, int y, std::string msg);
void dCircle(cv_bridge::CvImagePtr imgpt, int x, int y, cv::Scalar clr, int r);	//Draw circle at track point
void dRect(cv_bridge::CvImagePtr imgpt, int x, int y, int w, int h);	//Draw rect (bounding area)
void processImg(cv_bridge::CvImagePtr img, std::shared_ptr<vilib::Frame> ff, int image_width_, int image_height_);

// === DYNAMIC RECONFIG ===
void setDRVals(vilib_ros::tracker_paramConfig& config, bool debug);

// === CALLBACK & PUBLISHER ===
void dr_callback(vilib_ros::tracker_paramConfig& config, uint32_t level);
void pub_img(cv_bridge::CvImagePtr ipt);
void imgCallback(const sensor_msgs::ImageConstPtr& imgp);
