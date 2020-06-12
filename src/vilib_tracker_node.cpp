/*
 * ROS Node wrapper for for CUDA Visual Library by RPG. 
 * vilib_tracker_node.cpp
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

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>

#include "vilib_ros/keypt.h"
#include "vilib_ros/fast_paramConfig.h"

#include "vilib/cuda_common.h"
#include "vilib/preprocess/pyramid.h"
#include "vilib/storage/pyramid_pool.h"
#include "vilib/feature_detection/fast/fast_gpu.h"

using namespace vilib;

// Frame options
#define FRAME_IMAGE_PYRAMID_LEVELS 5
// Feature detection options
#define FEATURE_DETECTOR_CELL_SIZE_WIDTH 32
#define FEATURE_DETECTOR_CELL_SIZE_HEIGHT 32
#define FEATURE_DETECTOR_MIN_LEVEL 0
#define FEATURE_DETECTOR_MAX_LEVEL 2
#define FEATURE_DETECTOR_HORIZONTAL_BORDER 8
#define FEATURE_DETECTOR_VERTICAL_BORDER 8

// FAST parameters
#define FEATURE_DETECTOR_FAST_EPISLON 30.f
#define FEATURE_DETECTOR_FAST_ARC_LENGTH 10
#define FEATURE_DETECTOR_FAST_SCORE SUM_OF_ABS_DIFF_ON_ARC

// Pub/Sub
ros::Publisher ptsPub;
image_transport::Publisher imgPub;
image_transport::Subscriber imgSub;

//Tracker
std::size_t total_tracked_ftr_cnt, total_detected_ftr_cnt;


// === FEATURE TRACKER ===

//Feature Tracker fx, return all the points detected
std::shared_ptr<Frame> iTracker(cv_bridge::CvImagePtr imgpt, int image_width_, int image_height_)
{
    std::shared_ptr<vilib::DetectorBaseGPU> detector_gpu_;
std::shared_ptr<vilib::FeatureTrackerBase> tracker_gpu_;

     // Instantiation of the trackers
    vilib::FeatureTrackerOptions feature_tracker_options;
    feature_tracker_options.reset_before_detection = false;
    feature_tracker_options.use_best_n_features = 50;
    feature_tracker_options.min_tracks_to_detect_new_features = 0.3 * feature_tracker_options.use_best_n_features;
    feature_tracker_options.affine_est_gain = false;
    feature_tracker_options.affine_est_offset = false;

    // Create feature detector & tracker for the GPU
    detector_gpu_.reset(new FASTGPU(image_width_,
        image_height_,
        FEATURE_DETECTOR_CELL_SIZE_WIDTH,
        FEATURE_DETECTOR_CELL_SIZE_HEIGHT,
        FEATURE_DETECTOR_MIN_LEVEL,
        FEATURE_DETECTOR_MAX_LEVEL,
        FEATURE_DETECTOR_HORIZONTAL_BORDER,
        FEATURE_DETECTOR_VERTICAL_BORDER,
        FEATURE_DETECTOR_FAST_EPISLON,
        FEATURE_DETECTOR_FAST_ARC_LENGTH,
        FEATURE_DETECTOR_FAST_SCORE));
    tracker_gpu_.reset(new FeatureTrackerGPU(feature_tracker_options, 1));
    tracker_gpu_->setDetectorGPU(detector_gpu_, 0);

    // Initialize the pyramid pool
    vilib::PyramidPool::init(1,
        image_width_,
        image_height_,
        1, // grayscale
        FRAME_IMAGE_PYRAMID_LEVELS,
        IMAGE_PYRAMID_MEMORY_TYPE);

    // Create Frame
    std::shared_ptr<vilib::Frame> frame = std::make_shared<vilib::Frame>(
        img,
        0,
        vilib::FRAME_IMAGE_PYRAMID_LEVELS);
    // Create FrameBundle
    std::vector<std::shared_ptr<vilib::Frame> > framelist;
    framelist.push_back(frame);
    std::shared_ptr<vilib::FrameBundle> framebundle(new vilib::FrameBundle(framelist));
    tracker_gpu_->track(framebundle,
        total_tracked_ftr_cnt,
        total_detected_ftr_cnt);

    return frame;
}

// === GRAPHICS ===

// x, y: starting xy position for text
void drawText(cv_bridge::CvImagePtr imgpt, int x, int y, std::string msg)
{
    cv::putText(imgpt->image, msg, cv::Point(x, y), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8,
        cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
}

//Draw circle
void dCircle(cv_bridge::CvImagePtr imgpt, int x, int y)
{
    int thickness{ 2 };
    cv::circle(imgpt->image,
        cv::Point(x, y),
        1 * 3 * 1024,
        cv::Scalar(0, 255, 0),
        thickness,
        8,
        10);
    //ROS_WARN_STREAM("CC: " << img.channels(););

    //cv::imshow("cir",img);
    //      cv::waitKey(5);
    // cv::destroyWindow("cir");
}

//Draw rect (bounding area)
void dRect(cv_bridge::CvImagePtr imgpt, int x, int y, int w, int h){
int thickness = 2;
cv::Rect rect(x, y, w, h);
cv::rectangle(imgpt->image, rect, cv::Scalar(0, 255, 0), thickness);
}

//Draw text & detected features on img
void processImg(cv_bridge::CvImagePtr img, std::unordered_map<int, int> pts, int image_width_, int image_height_)
{
    //Points to publish
    vilib_ros::keypt pt_msg;
    pt_msg.stamp = ros::Time::now();
    pt_msg.size = pts.size();

//Draw detctor bounding area
   dRect(
img, 
image_width_/2 - (image_width_-2*HORIZONTAL_BORDER)/2, 
image_height_/2 - (image_height_-2*VERTICAL_BORDER)/2,
image_width_-2*HORIZONTAL_BORDER,
image_height_-2*VERTICAL_BORDER
); 

    // draw circles for the identified keypoints
    for (auto it = pts.begin(); it != pts.end(); ++it) {
        int x{ it->first & 0xFFFF };
        int y{ ((it->first >> 16) & 0xFFFF) };
        //std::cout << "x: " << x << " , y: " << y << std::endl;
        int thickness{ 1 };
        if (it->second == 3) {
            //Add the points
            geometry_msgs::Point pt;
            pt.x = x;
            pt.y = y;
            pt_msg.points.push_back(pt);
            //Draw the points
            dCircle(img, x * 1024, y * 1024);
        }
    }

    ptsPub.publish(pt_msg);

    //Draw text on img
    std::string tPoints{ "Corners: " + std::to_string(pts.size()) };
   drawText(img, 30, 30, tPoints);
}

// === DYNAMIC RECONFIG ===

void setDRVals(vilib_ros::fast_paramConfig& config, bool debug)
{
    PYRAMID_LEVELS = config.PYRAMID_LEVELS; //Pyramid level
    PYRAMID_MAX_LEVEL = PYRAMID_LEVELS;
    FAST_SCORE = (config.FAST_SCORE ? vilib::MAX_THRESHOLD : vilib::SUM_OF_ABS_DIFF_ON_ARC);

    FAST_EPSILON = (float)config.FAST_EPSILON; //Threshold level
    FAST_MIN_ARC_LENGTH = config.FAST_MIN_ARC_LENGTH;

    //NMS
    HORIZONTAL_BORDER = config.HORIZONTAL_BORDER;
    VERTICAL_BORDER = config.VERTICAL_BORDER;
    CELL_SIZE_WIDTH = config.CELL_SIZE_WIDTH;
    CELL_SIZE_HEIGHT = config.CELL_SIZE_HEIGHT;

    if (debug) {
        ROS_WARN("Reconfigure Request: %d %f %d", config.PYRAMID_LEVELS, config.FAST_EPSILON, config.FAST_SCORE);
    }
}

// === CALLBACK & PUBLISHER ===

void pub_img(cv_bridge::CvImagePtr ipt)
{
    //Publisher
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr16", ipt->image).toImageMsg();
    imgPub.publish(msg); //Publish image
}

//Dynamic reconfigure
void dr_callback(vilib_ros::fast_paramConfig& config, uint32_t level)
{
    setDRVals(config, false);
}

void imgCallback(const sensor_msgs::ImageConstPtr& imgp)
{
    try {
        std::unordered_map<int, int> pts; //Feature points detected

        //http://docs.ros.org/kinetic/api/sensor_msgs/html/image__encodings_8h_source.html
        cv_bridge::CvImagePtr imagePtrRaw{ cv_bridge::toCvCopy(imgp, sensor_msgs::image_encodings::BGR16) };
        //cv::imshow("view", cv_bridge::toCvShare(`imgp, "bgr16")->image);
        //cv::waitKey(30);

        cv_bridge::CvImagePtr gImg{ cv_bridge::toCvCopy(imgp, sensor_msgs::image_encodings::MONO8) }; //Create tmp img for detctor

        //Get image width & height
        int image_width_{ gImg->image.cols };
        int image_height_{ gImg->image.rows };

        pts = fDetector(gImg, image_width_, image_height_); //Feature detector (FAST) with grayscale img

        processImg(imagePtrRaw, pts, image_width_, image_height_); //Draw the feature point(s)on the img/vid

        //Publisher thread
        std::thread img_th(pub_img, imagePtrRaw);
        img_th.join();
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr16'.", imgp->encoding.c_str());
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vilib_ros");
    ros::NodeHandle nh;

    //Start Multithreading Process(Async thread): http://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning
    ros::AsyncSpinner spinner(4);
    ros::Rate r(120); //Run at 120Hz

    spinner.start();

    //Dynamic reconfig
    dynamic_reconfigure::Server<vilib_ros::fast_paramConfig> ft_server;
    dynamic_reconfigure::Server<vilib_ros::fast_paramConfig>::CallbackType ft_cb;

    ft_cb = boost::bind(&dr_callback, _1, _2);
    ft_server.setCallback(ft_cb);

    // Pub-Sub
    image_transport::ImageTransport it(nh);
    imgSub = it.subscribe("img_in", 1, imgCallback); //Sub

    ptsPub = nh.advertise<vilib_ros::keypt>("fast_pts", 1);
    imgPub = it.advertise("img_out", 1);

    ros::waitForShutdown();

    return 0;
}

