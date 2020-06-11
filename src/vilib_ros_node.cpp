#include <iostream>
#include <vector>
#include <unordered_map>
#include <thread>
#include <future>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "vilib_ros/keypt.h"

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

// Frame preprocessing
#define PYRAMID_LEVELS 6
#define PYRAMID_MIN_LEVEL 0
#define PYRAMID_MAX_LEVEL PYRAMID_LEVELS

// FAST detector parameters
#define FAST_EPSILON (60.0f) //Threshold level
#define FAST_MIN_ARC_LENGTH 10
// Remark: the Rosten CPU version only works with
//         SUM_OF_ABS_DIFF_ON_ARC and MAX_THRESHOLD
#define FAST_SCORE SUM_OF_ABS_DIFF_ON_ARC

// NMS parameters
#define HORIZONTAL_BORDER 0
#define VERTICAL_BORDER 0
#define CELL_SIZE_WIDTH 16
#define CELL_SIZE_HEIGHT 16

image_transport::Publisher imgPub;
ros::Publisher ptsPub;

image_transport::Subscriber imgSub;

//Point struct
struct Pt {
    int32_t x;
    int32_t y;
};

// === FEATURE DETECTOR ===

//FASt corner detector fx, return all the points detected
std::unordered_map<int, int> fDetector(cv_bridge::CvImagePtr imgpt)
{
    std::shared_ptr<vilib::DetectorBaseGPU> detector_gpu_;

    //For pyramid storage
    int image_width_ = imgpt->image.cols;
    int image_height_ = imgpt->image.rows;

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
    auto& points_gpu = detector_gpu_->getPoints();
    auto& points_gpu_grid = detector_gpu_->getGrid();

    std::unordered_map<int, int> points_combined;
    points_combined.reserve(points_gpu.size());

    int qqq = 0; //Index tracker
    for (auto it = points_gpu.begin(); it != points_gpu.end(); ++it) {
        int key = ((int)it->x_) | (((int)it->y_) << 16);
        if (key) {
            points_combined.emplace(key, 3);
            //ROS_WARN_STREAM("#" << qqq << " , key: " << key);
            //ROS_WARN_STREAM("x: " << it->x_ << " , y: " << it->y_);
        }
        ++qqq;
    }

    ROS_WARN_STREAM("All points: " << points_gpu.size() << ", Valid points: " << points_combined.size() << ", Epsilon: " << FAST_EPSILON);

    PyramidPool::deinit(); // Deinitialize the pyramid pool (for consecutive frames)

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
    int thickness = 2;
    cv::circle(img,
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

    return img;
}

//Draw text & detected features on img
cv_bridge::CvImagePtr processImg(cv_bridge::CvImagePtr img, std::unordered_map<int, int> pts)
{
    //Points to publish
    vilib_ros::keypt pt_msg;
    pt_msg.stamp = ros::Time::now();
    pt_msg.size = pts.size();

    // draw circles for the identified keypoints
    for (auto it = pts.begin(); it != pts.end(); ++it) {
        int x = it->first & 0xFFFF;
        int y = ((it->first >> 16) & 0xFFFF);
        //std::cout << "x: " << x << " , y: " << y << std::endl;
        int thickness = 1;
        if (it->second == 3) {
            //Add the points
            geometry_msgs::Point pt;
            pt.x = x;
            pt.y = y;
            pt_msg.points.push_back(pt);
            //Draw the points
            img->image = dCircle(img->image, x * 1024, y * 1024);
        }
    }

    ptsPub.publish(pt_msg);

    //Draw text on img
    std::string tPoints = "Corners: " + std::to_string(pts.size());
    img->image = drawText(img->image, 30, 30, tPoints);

    return img;
}

void pub_img(cv_bridge::CvImagePtr ipt){
//Publisher
 sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr16", ipt->image).toImageMsg();
        imgPub.publish(msg); //Publish image
}


void imgCallback(const sensor_msgs::ImageConstPtr& imgp)
{
    try {
        std::unordered_map<int, int> pts; //Feature points detected

        //http://docs.ros.org/kinetic/api/sensor_msgs/html/image__encodings_8h_source.html
        cv_bridge::CvImagePtr imagePtrRaw = cv_bridge::toCvCopy(imgp, sensor_msgs::image_encodings::BGR16);
        //cv::imshow("view", cv_bridge::toCvShare(`imgp, "bgr16")->image);
        //cv::waitKey(30);

        cv_bridge::CvImagePtr gImg = cv_bridge::toCvCopy(imgp, sensor_msgs::image_encodings::MONO8); //Create tmp img for detctor


auto tmp_pts = async(fDetector, gImg);
pts = tmp_pts.get();

       // pts = fDetector(gImg); //Feature detector (FAST) with grayscale img

        imagePtrRaw = processImg(imagePtrRaw, pts); //Draw the feature point(s)on the img/vid

        //Publisher
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
  ros::Rate r(120);  //Run at 120Hz

  spinner.start();

    image_transport::ImageTransport it(nh);

    imgSub = it.subscribe("img_in", 1, imgCallback); //Sub

    //Publisher
    ptsPub = nh.advertise<vilib_ros::keypt>("fast_pts", 1);
    imgPub = it.advertise("img_out", 1);

    //sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    //sensor_msgs::ImagePtr msg;

ros::waitForShutdown();

    return 0;
}
