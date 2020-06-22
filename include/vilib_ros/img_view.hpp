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

#include <vector>
#include <unordered_map>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/cudev/common.hpp>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include "vilib/cuda_common.h"
#include "vilib/preprocess/pyramid.h"
#include "vilib/storage/pyramid_pool.h"
#include "vilib/feature_detection/fast/fast_gpu.h"

#include "vilib_ros/keypt.h"

class ImgView {
public:
    ImgView(const ros::NodeHandle& nh_);
    void init(); //Init
    // === CALLBACK & PUBLISHER ===
    void ptsCallback(const vilib_ros::keyptConstPtr& pts);
  //  void imgCallback(const sensor_msgs::ImageConstPtr& imgp);
void pub_img(const cv_bridge::CvImagePtr& ipt);

private:
    ros::NodeHandle nh; //Node handle
    // Pub/Sub
    ros::Publisher ptsPub;
ros::Subscriber ptsSub;
    image_transport::Publisher imgPub;
   // image_transport::Subscriber imgSub;
	int frameID_track{-1};		//Used to sync img and feature pts

//Keypoints
vilib_ros::keypt keypt_list;

    // === GRAPHICS ===
    void drawText(const cv_bridge::CvImagePtr& imgpt, const int& x, const int& y, const std::string& msg);
    void dCircle(const cv_bridge::CvImagePtr& imgpt, const int& x, const int& y, const cv::Scalar& clr, const int& thickness); //Draw circle at track point
    void dRect(const cv_bridge::CvImagePtr& imgpt, const int& x, const int& y, const int& w, const int& h, const int& thickness); //Draw rect (bounding area)
    void processImg(const cv_bridge::CvImagePtr& img, const vilib_ros::keypt& plist, const int& image_width_, const int& image_height_);

};

