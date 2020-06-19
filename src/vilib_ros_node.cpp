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
#include "vilib_ros/config/config_fast.hpp"

using namespace vilib;

VFASTNode::VFASTNode(const ros::NodeHandle& nh_)
{
    this->nh = nh_;
};

void VFASTNode::init()
{
    // Pub-Sub
    image_transport::ImageTransport it(this->nh);
    imgSub = it.subscribe("img_in", 1, &VFASTNode::imgCallback, this); //Sub

    ptsPub = nh.advertise<vilib_ros::keypt>("fast_pts", 1);
    imgPub = it.advertise("img_out", 1);

    startReconfig();
}

// === CALLBACK & PUBLISHER ===
void VFASTNode::dr_callback(const vilib_ros::fast_paramConfig& config, const uint32_t& level)
{
    setDRVals(config, false);
//Reset GPU
init_=false;
}

void VFASTNode::pub_img(const cv_bridge::CvImagePtr& ipt)
{
//    sensor_msgs::ImagePtr msg{ cv_bridge::CvImage(std_msgs::Header(), "bgr16", ipt->image).toImageMsg() };
    imgPub.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr16", ipt->image).toImageMsg());
}

void VFASTNode::imgCallback(const sensor_msgs::ImageConstPtr& imgp)
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
	gImg->image.release();

	const auto psimg_out = std::async(std::launch::async, &VFASTNode::processImg, this, imagePtrRaw, pts, image_width_, image_height_);
        //processImg(imagePtrRaw, pts, image_width_, image_height_); //Draw the feature point(s)on the img/vid
	
        //pub_img(imagePtrRaw);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr16'.", imgp->encoding.c_str());
    }
}
std::shared_ptr<vilib::DetectorBaseGPU> detector_gpu_;
std::shared_ptr<vilib::DetectorBaseGPU> VFASTNode::fDetector(const cv_bridge::CvImagePtr& imgpt, const int& image_width_, const int& image_height_)
{

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

if(!init_){
    // Initialize the pyramid pool
    vilib::PyramidPool::init(1,
        image_width_,
        image_height_,
        1, // grayscale
        PYRAMID_LEVELS,
        IMAGE_PYRAMID_MEMORY_TYPE);

init_=true;
}

    std::shared_ptr<vilib::Frame> frame0(new vilib::Frame(imgpt->image, 0, PYRAMID_LEVELS)); // Create a Frame (image upload, pyramid)
    detector_gpu_->reset(); // Reset detector's grid (Note: this step could be actually avoided with custom processing)
    detector_gpu_->detect(frame0->pyramid_); // Do the detection

    vilib::PyramidPool::deinit(); // Deinitialize the pyramid pool

return detector_gpu_;
}

std::unordered_map<int, int> VFASTNode::getPoints(const std::shared_ptr<vilib::DetectorBaseGPU>& detector_gpu_){
  // Display results
    auto& points_gpu{ detector_gpu_->getPoints() };
    auto& points_gpu_grid{ detector_gpu_->getGrid() };

    std::unordered_map<int, int> points_combined;
    points_combined.reserve(points_gpu.size());

    int pidx{ 0 }; //Index tracker
    for (auto it = points_gpu.begin(); it != points_gpu.end(); ++it) {
        int key = ((int)it->x_) | (((int)it->y_) << 16);
        if (key) {
	//ROS_WARN_STREAM(it->x_<<","<<it->y_<<","<<it->score_<<","<<it->level_);
            points_combined.emplace(key, 3);
        }
        ++pidx;
    }
    return points_combined;
}

// === GRAPHICS ===
void VFASTNode::drawText(const cv_bridge::CvImagePtr& imgpt, const int& x, const int& y, const std::string& msg)
{
    cv::putText(imgpt->image, msg, cv::Point(x, y), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8,
        cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
}

void VFASTNode::dCircle(const cv_bridge::CvImagePtr& imgpt, const int& x, const int& y, const int& thickness)
{
    cv::circle(imgpt->image,
        cv::Point(x, y),
        1 * 3 * 1024,
        cv::Scalar(0, 255, 0),
        thickness,
        8,
        10);
}

void VFASTNode::dRect(const cv_bridge::CvImagePtr& imgpt, const int& x, const int& y, const int& w, const int& h, const int& thickness)
{
    cv::Rect rect(x, y, w, h);
    cv::rectangle(imgpt->image, rect, cv::Scalar(0, 255, 0), thickness);
}

void VFASTNode::processImg(const cv_bridge::CvImagePtr& img, const std::unordered_map<int, int>& pts, const int& image_width_, const int& image_height_)
{
    //Points to publish
    vilib_ros::keypt pt_msg;
    pt_msg.stamp = ros::Time::now();
    pt_msg.size = pts.size();

std::lock_guard<std::mutex> lock(pimgMutex);

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
            dCircle(img, x * 1024, y * 1024, 2);
        }
    }

    ptsPub.publish(pt_msg); //Publish /feature_pts

 //Draw detection bounding area
    dRect(
        img,
        HORIZONTAL_BORDER,
        VERTICAL_BORDER,
        image_width_ - 2 * HORIZONTAL_BORDER,
        image_height_ - 2 * VERTICAL_BORDER,
	2);

    //Draw text on img
    std::string tPoints{ "Corners: " + std::to_string(pts.size()) };
    drawText(img, 30, 30, tPoints);

    pub_img(img);
 img->image.release();
}

// === DYNAMIC RECONFIG ===
void VFASTNode::setDRVals(const vilib_ros::fast_paramConfig& config, const bool& debug)
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

void VFASTNode::startReconfig()
{
    ft_cb = boost::bind(&VFASTNode::dr_callback, this, _1, _2);
    ft_server.setCallback(ft_cb);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vilib_ros");
    ros::NodeHandle nh;

    VFASTNode vrf(nh);
    vrf.init();

    //Start Multithreading Process(Async thread): http://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning
    ros::AsyncSpinner spinner(4);
    ros::Rate r(30); //Run at 30Hz

    spinner.start();
    ros::waitForShutdown();
    vilib::PyramidPool::deinit(); // Deinitialize the pyramid pool
    return 0;
}

