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
#include <future>

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

#include "vilib_ros/keypt.h"
#include "vilib_ros/fast_paramConfig.h"

class VFASTNode {
public:
    VFASTNode(const ros::NodeHandle& nh_);
    //Init
    void init();
    // === CALLBACK & PUBLISHER ===
    void dr_callback(const vilib_ros::fast_paramConfig& config, const uint32_t& level); //Dynamic reconfigure
    void pub_img(const cv_bridge::CvImagePtr& ipt); //Publish img with features
    void imgCallback(const sensor_msgs::ImageConstPtr& imgp); //Image Input callback

private:

    std::mutex pimgMutex;
    ros::NodeHandle nh;		//Node handle
    // Pub/Sub
    ros::Publisher ptsPub;
    image_transport::Publisher imgPub;
    image_transport::Subscriber imgSub;
    //Dynamic reconfig
    dynamic_reconfigure::Server<vilib_ros::fast_paramConfig> ft_server;
    dynamic_reconfigure::Server<vilib_ros::fast_paramConfig>::CallbackType ft_cb;

    // === FEATURE DETECTOR ===
    bool init_{ false };
    std::shared_ptr<vilib::DetectorBaseGPU> fDetector(const cv_bridge::CvImagePtr& imgpt, const int& image_width_, const int& image_height_); 
    std::unordered_map<int, int> getPoints(const std::shared_ptr<vilib::DetectorBaseGPU>& detector_gpu_);

    // === GRAPHICS ===
    void drawText(const cv_bridge::CvImagePtr& imgpt, const int& x, const int& y, const std::string& msg); //Draw text
    void dCircle(const cv_bridge::CvImagePtr& imgpt, const int& x, const int& y, const int& thickness); //Draw feature point
    void dRect(const cv_bridge::CvImagePtr& imgpt, const int& x, const int& y, const int& w, const int& h, const int& thickness); //Draw bounding rectangle
    void processImg(const cv_bridge::CvImagePtr& img, const std::unordered_map<int, int>& pts, const int& image_width_, const int& image_height_); //Draw text & detected features on img

    // === DYNAMIC RECONFIG ===
    void setDRVals(const vilib_ros::fast_paramConfig& config, const bool& debug);
    void startReconfig();
};
