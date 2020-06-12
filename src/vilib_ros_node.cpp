/*
 * ROS Node wrapper for for CUDA Visual Library by RPG. 
 * vilib_ros_node.cpp
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

// Frame preprocessing
int PYRAMID_LEVELS{ 6 };
constexpr int PYRAMID_MIN_LEVEL{ 0 };
int PYRAMID_MAX_LEVEL{ PYRAMID_LEVELS };

// FAST detector parameters
float FAST_EPSILON{ 60.0f }; //Threshold level
int FAST_MIN_ARC_LENGTH{ 10 };
// Remark: the Rosten CPU version only works with
//         SUM_OF_ABS_DIFF_ON_ARC and MAX_THRESHOLD
vilib::fast_score FAST_SCORE{ SUM_OF_ABS_DIFF_ON_ARC };

// NMS parameters
int HORIZONTAL_BORDER{ 0 }; //Horizontal image detection padding (Act as clip off for detection)
int VERTICAL_BORDER{ 0 }; //Vertical image detection padding
int CELL_SIZE_WIDTH{ 16 };
int CELL_SIZE_HEIGHT{ 16 };

// Pub/Sub
ros::Publisher ptsPub;
image_transport::Publisher imgPub;
image_transport::Subscriber imgSub;

// === FEATURE DETECTOR ===

//FASt corner detector fx, return all the points detected
std::unordered_map<int, int> fDetector(cv_bridge::CvImagePtr imgpt, int image_width_, int image_height_)
{
    std::shared_ptr<vilib::DetectorBaseGPU> detector_gpu_;

    //For pyramid storage
    detector_gpu_.reset(new FASTGPU(image_width_,
        image_height_,
        CELL_SIZE_WIDTH,
        CELL_SIZE_HEIGHT,
        PYRAMID_MIN_LEVEL,
        PYRAMID_MAX_LEVEL,
        HORIZONTAL_BORDER,
        VERTICAL_BORDER,
        FAST_EPSILON,
        FAST_MIN_ARC_LENGTH,
        FAST_SCORE));

    // Initialize the pyramid pool
    PyramidPool::init(1,
        image_width_,
        image_height_,
        1, // grayscale
        PYRAMID_LEVELS,
        IMAGE_PYRAMID_MEMORY_TYPE);

    std::shared_ptr<Frame> frame0(new Frame(imgpt->image, 0, PYRAMID_LEVELS)); // Create a Frame (image upload, pyramid)
    detector_gpu_->reset(); // Reset detector's grid (Note: this step could be actually avoided with custom processing)
    detector_gpu_->detect(frame0->pyramid_); // Do the detection

    // Display results
    auto& points_gpu{ detector_gpu_->getPoints() };
    auto& points_gpu_grid{ detector_gpu_->getGrid() };

    std::unordered_map<int, int> points_combined;
    points_combined.reserve(points_gpu.size());

    int qqq{ 0 }; //Index tracker
    for (auto it = points_gpu.begin(); it != points_gpu.end(); ++it) {
        int key = ((int)it->x_) | (((int)it->y_) << 16);
        if (key) {
            points_combined.emplace(key, 3);
            //ROS_WARN_STREAM("#" << qqq << " , key: " << key);
            //ROS_WARN_STREAM("x: " << it->x_ << " , y: " << it->y_);
        }
        ++qqq;
    }

    //ROS_WARN_STREAM("All points: " << points_gpu.size() << ", Valid points: " << points_combined.size() << ", Epsilon: " << FAST_EPSILON);
    PyramidPool::deinit(); // Deinitialize the pyramid pool (for consecutive frames)

    return points_combined;
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
cv::Rect rect(x, y, w, h);
cv::rectangle(imgpt->image, rect, cv::Scalar(0, 255, 0));
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
