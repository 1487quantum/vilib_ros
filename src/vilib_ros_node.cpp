#include <iostream>
#include <vector>
#include <unordered_map>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "vilib/cuda_common.h"
#include "vilib/preprocess/pyramid.h"
#include "vilib/storage/pyramid_pool.h"
#include "vilib/feature_detection/fast/fast_gpu.h"

/*
class vilib_ros {
public:
    void imgCallback(const sensor_msgs::ImageConstPtr& img1, const sensor_msgs::ImageConstPtr& img2)
    {
        cv_bridge::CvImagePtr imagePtr1, imagePtr2;

        imagePtr1 = cv_bridge::toCvCopy(img1, sensor_msgs::image_encodings::MONO8);
        std::cout << "Left camera: ";

        imagePtr2 = cv_bridge::toCvCopy(img2, sensor_msgs::image_encodings::MONO8);
        std::cout << "Right camera: ";
    }

private:
    ros::NodeHandle nh;
};
*/

using namespace vilib;

std::shared_ptr<vilib::DetectorBaseGPU> detector_gpu_;

// Frame preprocessing
#define PYRAMID_LEVELS 1
#define PYRAMID_MIN_LEVEL 0
#define PYRAMID_MAX_LEVEL PYRAMID_LEVELS

// FAST detector parameters
#define FAST_EPSILON (10.0f) //Threshold level
#define FAST_MIN_ARC_LENGTH 10
// Remark: the Rosten CPU version only works with
//         SUM_OF_ABS_DIFF_ON_ARC and MAX_THRESHOLD
#define FAST_SCORE SUM_OF_ABS_DIFF_ON_ARC

// NMS parameters
#define HORIZONTAL_BORDER 0
#define VERTICAL_BORDER 0
#define CELL_SIZE_WIDTH 32
#define CELL_SIZE_HEIGHT 32

std::unordered_map<int, int> pts; //Feature points detected
// === FEATURE DETECTOR ===

//FASt corner detector fx, return all the points detected
std::unordered_map<int, int> fDetector(cv::Mat img)
{
    //For pyramid storage
    int image_width_ = img.cols;
    int image_height_ = img.rows;

    //std::cout << "Detector (GPU) reset" << std::endl;
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

    // Create a Frame (image upload, pyramid)
    std::shared_ptr<Frame> frame0(new Frame(img, 0, PYRAMID_LEVELS));
    // Reset detector's grid
    // Note: this step could be actually avoided with custom processing
    detector_gpu_->reset();
    // Do the detection
    detector_gpu_->detect(frame0->pyramid_);

    // Display results
    auto& points_gpu = detector_gpu_->getPoints();
    auto& points_gpu_grid = detector_gpu_->getGrid();

    std::unordered_map<int, int> points_combined;
    points_combined.reserve(points_gpu.size());

    int qqq = 0; //Index tracker
    for (auto it = points_gpu.begin(); it != points_gpu.end(); ++it) {
        int key = ((int)it->x_) | (((int)it->y_) << 16);

        if (key) {
            //std::cout << "#" << qqq << " , key: " << key << std::endl;
            points_combined.emplace(key, 3);
        }
        else {
            points_combined.emplace(key, 1);
        }
        qqq++;
    }
    std::cout << "All points: " << points_gpu.size() << ", Valid points: " << points_combined.size() << ", Epsilon: " << FAST_EPSILON << std::endl;
    ROS_WARN_STREAM("All points: " << points_gpu.size() << ", Valid points: " << points_combined.size() << ", Epsilon: " << FAST_EPSILON);
    // Deinitialize the pyramid pool (for consecutive frames)
    PyramidPool::deinit();

    return points_combined;
}

// === GRAPHICS ===

// x, y: starting xy position for text
cv::Mat drawText(cv::Mat img, int x, int y, std::string msg)
{
    cv::putText(img, msg, cv::Point(x, y), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8,
        cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
    return img;
}

//Draw circle
cv::Mat dCircle(cv::Mat img, int x, int y)
{
    int thickness = 1;
    cv::circle(img,
        cv::Point(x, y),
        1 * 3 * 1024,
        cv::Scalar(0, 255, 255),
        thickness,
        8,
        10);
    return img;
}

//Draw text & detected features on img
cv::Mat processImg(cv::Mat img, std::unordered_map<int, int> pts)
{
    cv::cvtColor(img, img, cv::COLOR_GRAY2BGR); //Convert to
    // draw circles for the identified keypoints
    for (auto it = pts.begin(); it != pts.end(); ++it) {
        int x = (it->first & 0xFFFF) * 1024;
        int y = ((it->first >> 16) & 0xFFFF) * 1024;
        //std::cout << "x: " << x << " , y: " << y << std::endl;
        cv::Scalar color; // B,G,R
        int thickness = 1;
        if (it->second == 3) {
            img = dCircle(img, x, y);
        }
    }

    //Draw text on img
    std::string tPoints = "Corners: " + std::to_string(pts.size());
    img = drawText(img, 30, 30, tPoints);

    return img;
}

image_transport::Publisher imgPub;
image_transport::Subscriber imgSub;

void imgCallback(const sensor_msgs::ImageConstPtr& imgp)
{
    try {
        //http://docs.ros.org/kinetic/api/sensor_msgs/html/image__encodings_8h_source.html
        cv_bridge::CvImagePtr imagePtrRaw = cv_bridge::toCvCopy(imgp, sensor_msgs::image_encodings::BGR16);
        std::cout << "Left camera: ";
        //cv::imshow("view", cv_bridge::toCvShare(img, "bgr8")->image);
        //cv::waitKey(30);

        cv::Mat gImg;
        cv::cvtColor(imagePtrRaw->image, gImg, cv::COLOR_BGR2GRAY); //Convert to grayscale for detector

        pts = fDetector(gImg); //Feature detector (FAST) with grayscale img
        gImg = processImg(gImg, pts); //Draw the feature point(s)on the img/vid

        //sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr16", imagePtrRaw->image).toImageMsg();
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr16", gImg).toImageMsg();

        imgPub.publish(msg);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr16'.", imgp->encoding.c_str());
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vilib_ros");
    ros::NodeHandle nh;

    // cv::namedWindow("view", cv::WINDOW_NORMAL);
    // cv::startWindowThread();

    image_transport::ImageTransport it(nh);

    //Publisher
    imgPub = it.advertise("img_out", 1);

    //Sub
    imgSub = it.subscribe("img_in", 1, imgCallback);

    //sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    sensor_msgs::ImagePtr msg;

    ros::spin();
    // cv::destroyWindow("view");

    return 0;
}

