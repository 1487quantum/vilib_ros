/*
 * ROS Node wrapper for for CUDA Visual Library by RPG. 
 * vilib_ros_node.cpp
 * FAST Detector ROS Node
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

#include "vilib_ros/vilib_ros_node.hpp"

using namespace vilib;

vilib_ros_fast::vilib_ros_fast(ros::NodeHandle nh_)
{
    this->nh = nh_;
};

void vilib_ros_fast::init()
{
    // Pub-Sub
    image_transport::ImageTransport it(this->nh);
    imgSub = it.subscribe("img_in", 1, &vilib_ros_fast::imgCallback, this); //Sub

    ptsPub = nh.advertise<vilib_ros::keypt>("fast_pts", 1);
    imgPub = it.advertise("img_out", 1);

    startReconfig();
}

// === CALLBACK & PUBLISHER ===
void vilib_ros_fast::dr_callback(vilib_ros::fast_paramConfig& config, uint32_t level)
{
    setDRVals(config, false);
}

void vilib_ros_fast::pub_img(cv_bridge::CvImagePtr ipt)
{
    sensor_msgs::ImagePtr msg{ cv_bridge::CvImage(std_msgs::Header(), "bgr16", ipt->image).toImageMsg() };
    imgPub.publish(msg);
}

void vilib_ros_fast::imgCallback(const sensor_msgs::ImageConstPtr& imgp)
{
    try {
        std::unordered_map<int, int> pts; //Feature points detected

        //http://docs.ros.org/kinetic/api/sensor_msgs/html/image__encodings_8h_source.html
        cv_bridge::CvImagePtr imagePtrRaw{ cv_bridge::toCvCopy(imgp, sensor_msgs::image_encodings::BGR16) };
        cv_bridge::CvImagePtr gImg{ cv_bridge::toCvCopy(imgp, sensor_msgs::image_encodings::MONO8) }; //Create tmp img for detctor

        //Get image width & height
        int image_width_{ gImg->image.cols };
        int image_height_{ gImg->image.rows };

        pts = getPoints(fDetector(gImg, image_width_, image_height_)); //Feature detector (FAST) with grayscale img
        processImg(imagePtrRaw, pts, image_width_, image_height_); //Draw the feature point(s)on the img/vid

        pub_img(imagePtrRaw);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr16'.", imgp->encoding.c_str());
    }
}

std::shared_ptr<vilib::DetectorBaseGPU> vilib_ros_fast::fDetector(cv_bridge::CvImagePtr imgpt, int image_width_, int image_height_)
{
std::shared_ptr<vilib::DetectorBaseGPU> detector_gpu_;
    //For pyramid storage
    detector_gpu_.reset(new vilib::FASTGPU(image_width_,
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
    vilib::PyramidPool::init(1,
        image_width_,
        image_height_,
        1, // grayscale
        PYRAMID_LEVELS,
        IMAGE_PYRAMID_MEMORY_TYPE);

    std::shared_ptr<vilib::Frame> frame0(new vilib::Frame(imgpt->image, 0, PYRAMID_LEVELS)); // Create a Frame (image upload, pyramid)
    detector_gpu_->reset(); // Reset detector's grid (Note: this step could be actually avoided with custom processing)
    detector_gpu_->detect(frame0->pyramid_); // Do the detection

    vilib::PyramidPool::deinit(); // Deinitialize the pyramid pool (for consecutive frames)

return detector_gpu_;
}

std::unordered_map<int, int> vilib_ros_fast::getPoints(std::shared_ptr<vilib::DetectorBaseGPU> detector_gpu_){
  // Display results
    auto& points_gpu{ detector_gpu_->getPoints() };
    auto& points_gpu_grid{ detector_gpu_->getGrid() };

    std::unordered_map<int, int> points_combined;
    points_combined.reserve(points_gpu.size());

    int pidx{ 0 }; //Index tracker
    for (auto it = points_gpu.begin(); it != points_gpu.end(); ++it) {
        int key = ((int)it->x_) | (((int)it->y_) << 16);
        if (key) {
            points_combined.emplace(key, 3);
        }
        ++pidx;
    }
    return points_combined;
}

// === GRAPHICS ===
void vilib_ros_fast::drawText(cv_bridge::CvImagePtr imgpt, int x, int y, const std::string& msg)
{
    cv::putText(imgpt->image, msg, cv::Point(x, y), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8,
        cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
}

void vilib_ros_fast::dCircle(cv_bridge::CvImagePtr imgpt, int x, int y)
{
    int thickness{ 2 };
    cv::circle(imgpt->image,
        cv::Point(x, y),
        1 * 3 * 1024,
        cv::Scalar(0, 255, 0),
        thickness,
        8,
        10);
}

void vilib_ros_fast::dRect(cv_bridge::CvImagePtr imgpt, int x, int y, int w, int h)
{
    int thickness{ 2 };
    cv::Rect rect(x, y, w, h);
    cv::rectangle(imgpt->image, rect, cv::Scalar(0, 255, 0), thickness);
}

void vilib_ros_fast::processImg(cv_bridge::CvImagePtr img, std::unordered_map<int, int> pts, int image_width_, int image_height_)
{
    //Points to publish
    vilib_ros::keypt pt_msg;
    pt_msg.stamp = ros::Time::now();
    pt_msg.size = pts.size();

    //Draw detection bounding area
    dRect(
        img,
        HORIZONTAL_BORDER,
        VERTICAL_BORDER,
        image_width_ - 2 * HORIZONTAL_BORDER,
        image_height_ - 2 * VERTICAL_BORDER);

    // draw circles for the identified keypoints
    for (auto it = pts.begin(); it != pts.end(); ++it) {
        int x{ it->first & 0xFFFF };
        int y{ ((it->first >> 16) & 0xFFFF) };
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

    ptsPub.publish(pt_msg); //Publish /feature_pts

    //Draw text on img
    std::string tPoints{ "Corners: " + std::to_string(pts.size()) };
    drawText(img, 30, 30, tPoints);
}

// === DYNAMIC RECONFIG ===
void vilib_ros_fast::setDRVals(vilib_ros::fast_paramConfig& config, bool debug)
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
        ROS_INFO("Reconfigure Request: %d %f %d", config.PYRAMID_LEVELS, config.FAST_EPSILON, config.FAST_SCORE);
    }
}

void vilib_ros_fast::startReconfig()
{
    ft_cb = boost::bind(&vilib_ros_fast::dr_callback, this, _1, _2);
    ft_server.setCallback(ft_cb);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vilib_ros");
    ros::NodeHandle nh;

    vilib_ros_fast vrf(nh);
    vrf.init();

    //Start Multithreading Process(Async thread): http://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning
    ros::AsyncSpinner spinner(4);
    ros::Rate r(60); //Run at 60Hz

    spinner.start();
    ros::waitForShutdown();

    return 0;
}

