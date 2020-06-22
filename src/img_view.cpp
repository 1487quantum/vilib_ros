/*
 * ROS Node wrapper for for CUDA Visual Library by RPG. 
 * vilib_ros_tracker.cpp
 * Feature points image viewer
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

#include "vilib_ros/img_view.hpp"


ImgView::ImgView(const ros::NodeHandle& nh_)
{
    this->nh = nh_;
};

void ImgView::init()
{
    // Pub-Sub
    image_transport::ImageTransport it(nh);
   // imgSub = it.subscribe("img_in", 1, &ImgView::imgCallback, this);
    ptsSub = nh.subscribe("feature_pts", 1, &ImgView::ptsCallback, this);
    imgPub = it.advertise("img_view_out", 1);

}

// === GRAPHICS ===
void ImgView::drawText(const cv_bridge::CvImagePtr& imgpt, const int& x, const int& y, const std::string& msg)
{
    cv::putText(imgpt->image, msg, cv::Point(x, y), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0,
        cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
}

void ImgView::dCircle(const cv_bridge::CvImagePtr& imgpt, const int& x, const int& y, const cv::Scalar& clr, const int& thickness)
{
    cv::circle(imgpt->image,
        cv::Point(x, y),
        1 * 3,
        clr,
        thickness,
        8 // line type
        ); 
}

void ImgView::dRect(const cv_bridge::CvImagePtr& imgpt, const int& x, const int& y, const int& w, const int& h, const int& thickness)
{
    cv::Rect rect(x, y, w, h);
    cv::rectangle(imgpt->image, rect, cv::Scalar(0, 255, 0), thickness);
}

void ImgView::processImg(const cv_bridge::CvImagePtr& img, const vilib_ros::keypt& plist, const int& image_width_, const int& image_height_)
{
    static int last_track_id{ -1 };
    static std::unordered_map<std::size_t, cv::Scalar> track_colors;

//TODO: Fix tracking issue, ref from ros_tracker


    // note: id-s start from 0
    for (int i = 0; i < plist.size; ++i) {
        // Track id
	const int& track_id{i};
	const int& pt_x = plist.points[i].x;
	const int& pt_y = plist.points[i].y;
        // Color: B,G,R
        cv::Scalar track_color(255, 255, 255);
        if (last_track_id < track_id) {
            // new feature: generate random color, but dont use is yet
            int channel_b{rand() % 255};
            int channel_g{rand() % 255};
            int channel_r{rand() % 255};
            track_colors[(std::size_t)track_id] = cv::Scalar(channel_b, channel_g, channel_r);
        }
        else {
            // tracked feature: use old color
            track_color = track_colors[track_id];
        }

        //ROS_WARN_STREAM( track_id << ": " << track_color<< ", x: "<<((int)x>>SHIFT_BITS)<<", y: "<<((int)y>>SHIFT_BITS));
        dCircle(img, pt_x, pt_y, track_color, 2); //Draw circle around feature

        
        //Label location offset
        int x_offset{ 0 };
        int y_offset{ -8 };
        //Draw pt number
        std::string ploc{ "P" + std::to_string(track_id) };
        drawText(img, pt_x + x_offset, pt_y + y_offset, ploc); //Shift coordinates back
    }

   ++last_track_id;

//Draw bounding area
/*
  dRect(
        img,
        FEATURE_DETECTOR_HORIZONTAL_BORDER,
        FEATURE_DETECTOR_VERTICAL_BORDER,
        image_width_ - 2 * FEATURE_DETECTOR_HORIZONTAL_BORDER,
        image_height_ - 2 * FEATURE_DETECTOR_VERTICAL_BORDER,
	3);
*/
    //Draw text on img
    std::string tPoints{ "Corners: " + std::to_string(plist.size) };
    drawText(img, 30, 30, tPoints);
pub_img(img);
}

void ImgView::ptsCallback(const vilib_ros::keyptConstPtr& pts)
{
//Set point vals
keypt_list.frame_id = pts->frame_id;
keypt_list.stamp = pts->stamp;
keypt_list.points = pts->points;
keypt_list.size = pts->size;
keypt_list.image_src = pts->image_src;
//ROS_WARN("%d %d",keypt_list.frame_id,keypt_list.size);

        cv_bridge::CvImagePtr imagePtrRaw{ cv_bridge::toCvCopy(keypt_list.image_src , sensor_msgs::image_encodings::BGR16) };

int image_width_{ imagePtrRaw->image.cols };
        int image_height_{ imagePtrRaw->image.rows };
        processImg(imagePtrRaw, keypt_list, image_width_, image_height_); //Draw the feature point(s)on the img/vid

}

void ImgView::pub_img(const cv_bridge::CvImagePtr& ipt)
{
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr16", ipt->image).toImageMsg();
    imgPub.publish(msg); //Publish image
    ipt->image.release();
}
/*
void ImgView::imgCallback(const sensor_msgs::ImageConstPtr& imgp)
{

    try {

        //http://docs.ros.org/kinetic/api/sensor_msgs/html/image__encodings_8h_source.html
        cv_bridge::CvImagePtr imagePtrRaw{ cv_bridge::toCvCopy(imgp, sensor_msgs::image_encodings::BGR16) };
        int image_width_{ imagePtrRaw->image.cols };
        int image_height_{ imagePtrRaw->image.rows };
        processImg(imagePtrRaw, keypt_list, image_width_, image_height_); //Draw the feature point(s)on the img/vid


    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr16'.", imgp->encoding.c_str());
    }
}
*/

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vilib_img_viewer");
    ros::NodeHandle nh;

    ImgView imv(nh);
    imv.init();

    ros::spin();

    return 0;
}

