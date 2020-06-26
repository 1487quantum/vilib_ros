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

#include "vilib_ros/keypt.h"
#include "vilib_ros/tracker_paramConfig.h"

class VTrackNode {
public:
    VTrackNode(const ros::NodeHandle& nh_);
    void init(); //Init
    // === CALLBACK & PUBLISHER ===
    void dr_callback(const vilib_ros::tracker_paramConfig& config, const uint32_t& level);
    void pub_img(const cv_bridge::CvImagePtr& ipt);
    void imgCallback(const sensor_msgs::ImageConstPtr& imgp);

private:
    ros::NodeHandle nh; //Node handle
    // Pub/Sub
    ros::Publisher ptsPub;
    image_transport::Publisher imgPub;
    image_transport::Subscriber imgSub;
    //Dynamic reconfig
    dynamic_reconfigure::Server<vilib_ros::tracker_paramConfig> ft_server;
    dynamic_reconfigure::Server<vilib_ros::tracker_paramConfig>::CallbackType ft_cb;

    // === FEATURE TRACKER ===
    std::shared_ptr<vilib::DetectorBaseGPU> detector_gpu_;
    std::shared_ptr<vilib::FeatureTrackerBase> tracker_gpu_;

    //Tracker
    bool initialized_{ false };
    std::size_t total_tracked_ftr_cnt, total_detected_ftr_cnt;
    std::shared_ptr<vilib::Frame> iTracker(const cv_bridge::CvImagePtr& imgpt, const int& image_width_, const int& image_height_); //Feature Tracker fx, return all the points detected

    // === GRAPHICS ===
    void drawText(const cv_bridge::CvImagePtr& imgpt, const int& x, const int& y, const std::string& msg);
    void dCircle(const cv_bridge::CvImagePtr& imgpt, const int& x, const int& y, const cv::Scalar& clr, const int& r, const int& thickness); //Draw circle at track point
    void dRect(const cv_bridge::CvImagePtr& imgpt, const int& x, const int& y, const int& w, const int& h, const int& thickness); //Draw rect (bounding area)
    void processImg(const cv_bridge::CvImagePtr& img, const std::shared_ptr<vilib::Frame>& ff, const int& image_width_, const int& image_height_);

    void startReconfig();
};

