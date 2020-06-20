/*
 * ROS Node wrapper for for CUDA Visual Library by RPG. 
 * vilib_ros_tracker.cpp
 * Lucas-Kanade Feature Tracker
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

#include "vilib_ros/vilib_ros_tracker.hpp"
#include "vilib_ros/config/config_tracker.hpp"

using namespace vilib;

VTrackNode::VTrackNode(const ros::NodeHandle& nh_)
{
    this->nh = nh_;
};

void VTrackNode::init()
{
    // Pub-Sub
    image_transport::ImageTransport it(nh);
    imgSub = it.subscribe("img_in", 1, &VTrackNode::imgCallback, this); //Sub

    ptsPub = nh.advertise<vilib_ros::keypt>("feature_pts", 1);
    imgPub = it.advertise("img_feature_out", 1);

    startReconfig();
}

std::shared_ptr<vilib::Frame> VTrackNode::iTracker(const cv_bridge::CvImagePtr& imgpt, const int& image_width_, const int& image_height_)
{
    if (!initialized_) {
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
        initialized_ = true;
    }

    // Create Frame
    std::shared_ptr<vilib::Frame> frame = std::make_shared<vilib::Frame>(
        imgpt->image,
        0,
        FRAME_IMAGE_PYRAMID_LEVELS);
    // Create FrameBundle
    std::vector<std::shared_ptr<vilib::Frame> > framelist;
    framelist.push_back(frame);
    std::shared_ptr<vilib::FrameBundle> framebundle(new vilib::FrameBundle(framelist));
    tracker_gpu_->track(framebundle,
        total_tracked_ftr_cnt,
        total_detected_ftr_cnt);

    // Deinitialize the pyramid pool (for consecutive frames)
    PyramidPool::deinit();

//Release Image
    imgpt->image.release();

    return frame;
}

// === GRAPHICS ===
void VTrackNode::drawText(const cv_bridge::CvImagePtr& imgpt, const int& x, const int& y, const std::string& msg)
{
    cv::putText(imgpt->image, msg, cv::Point(x, y), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0,
        cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
}

void VTrackNode::dCircle(const cv_bridge::CvImagePtr& imgpt, const int& x, const int& y, const cv::Scalar& clr, const int& r, const int& thickness)
{
    cv::circle(imgpt->image,
        cv::Point(x, y),
        1 * 3 * (1 << r),
        clr,
        thickness,
        8, // line type
        r); // shift: number of fractional bits in the coordinates AND the radius
}

void VTrackNode::dRect(const cv_bridge::CvImagePtr& imgpt, const int& x, const int& y, const int& w, const int& h, const int& thickness)
{
    cv::Rect rect(x, y, w, h);
    cv::rectangle(imgpt->image, rect, cv::Scalar(0, 255, 0), thickness);
}

void VTrackNode::processImg(const cv_bridge::CvImagePtr& img, const std::shared_ptr<vilib::Frame>& ff, const int& image_width_, const int& image_height_)
{
    //Feature Points to publish
    vilib_ros::keypt pt_msg;
    pt_msg.frame_id = ++frameID;			//To sync wih img frame
    pt_msg.stamp = ros::Time::now();
    pt_msg.size = ff->num_features_;

    static int last_track_id{ -1 };
    static std::unordered_map<std::size_t, cv::Scalar> track_colors;

    // note: id-s start from 0
    for (std::size_t i = 0; i < ff->num_features_; ++i) {
        const int SHIFT_BITS = 10;
        // Circle center
        const Eigen::Vector2d& pos_2d = ff->px_vec_.col(i);
        float x = pos_2d[0] * (1 << SHIFT_BITS);
        float y = pos_2d[1] * (1 << SHIFT_BITS);
        // Track id
        const int& track_id{ff->track_id_vec_[i]};
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
        

//dCircle(img, (int)x, (int)y, track_color, SHIFT_BITS, 2); //Draw circle around feature

        //Label points
        int pt_x{ ((int)x >> SHIFT_BITS) };
        int pt_y{ ((int)y >> SHIFT_BITS) };
        //Label location offset
        int x_offset{ 0 };
        int y_offset{ -8 };
        //Add the points
        geometry_msgs::Point pt;
        pt.x = pt_x;
        pt.y = pt_y;
        pt.z = track_id; //Z would store the feature Index
        pt_msg.points.push_back(pt);
/*
        //Draw pt number
        std::string ploc{ "P" + std::to_string(track_id) };
        drawText(img, pt_x + x_offset, pt_y + y_offset, ploc); //Shift coordinates back
*/
    }

    // update the highest track id
    if (ff->num_features_ > 0 && ff->track_id_vec_[ff->num_features_ - 1] > last_track_id) {
        last_track_id = ff->track_id_vec_[ff->num_features_ - 1];
    }

//Draw bounding area
  dRect(
        img,
        FEATURE_DETECTOR_HORIZONTAL_BORDER,
        FEATURE_DETECTOR_VERTICAL_BORDER,
        image_width_ - 2 * FEATURE_DETECTOR_HORIZONTAL_BORDER,
        image_height_ - 2 * FEATURE_DETECTOR_VERTICAL_BORDER,
	2);

/*
    //Draw text on img
    std::string tPoints{ "Corners: " + std::to_string(ff->num_features_) };
    drawText(img, 30, 30, tPoints);
*/
    //Publish features
    ptsPub.publish(pt_msg);

}

// === DYNAMIC RECONFIG ===
void VTrackNode::setDRVals(const vilib_ros::tracker_paramConfig& config, const bool& debug)
{
    // Frame options
    FRAME_IMAGE_PYRAMID_LEVELS = config.FRAME_IMAGE_PYRAMID_LEVELS;
    // Feature detection options
    FEATURE_DETECTOR_CELL_SIZE_WIDTH = config.FEATURE_DETECTOR_CELL_SIZE_WIDTH;
    FEATURE_DETECTOR_CELL_SIZE_HEIGHT = config.FEATURE_DETECTOR_CELL_SIZE_HEIGHT;
    FEATURE_DETECTOR_MIN_LEVEL = config.FEATURE_DETECTOR_MIN_LEVEL;
    FEATURE_DETECTOR_MAX_LEVEL = config.FEATURE_DETECTOR_MAX_LEVEL;
    FEATURE_DETECTOR_HORIZONTAL_BORDER = config.FEATURE_DETECTOR_HORIZONTAL_BORDER;
    FEATURE_DETECTOR_VERTICAL_BORDER = config.FEATURE_DETECTOR_VERTICAL_BORDER;
    // FAST parameters
    FEATURE_DETECTOR_FAST_EPISLON = (float)config.FEATURE_DETECTOR_FAST_EPISLON;
    FEATURE_DETECTOR_FAST_ARC_LENGTH = config.FEATURE_DETECTOR_FAST_ARC_LENGTH;
    FEATURE_DETECTOR_FAST_SCORE = (config.FEATURE_DETECTOR_FAST_SCORE ? vilib::MAX_THRESHOLD : vilib::SUM_OF_ABS_DIFF_ON_ARC);
    //Reset detector
    initialized_ = false;
    if (debug) {
        ROS_INFO("\nReconfigure Request:\n=== FRAME OPTIONS ===\n%d\n=== FEATURE DETECTION ===\n%d %d %d %d %d %d\n=== FAST PARAMETERS ===\n%f %d %d\n",
            config.FRAME_IMAGE_PYRAMID_LEVELS,
            config.FEATURE_DETECTOR_CELL_SIZE_WIDTH,
            config.FEATURE_DETECTOR_CELL_SIZE_HEIGHT,
            config.FEATURE_DETECTOR_MIN_LEVEL,
            config.FEATURE_DETECTOR_MAX_LEVEL,
            config.FEATURE_DETECTOR_HORIZONTAL_BORDER,
            config.FEATURE_DETECTOR_VERTICAL_BORDER,
            config.FEATURE_DETECTOR_FAST_EPISLON,
            config.FEATURE_DETECTOR_FAST_ARC_LENGTH,
            config.FEATURE_DETECTOR_FAST_SCORE);
    }
}

// === CALLBACK & PUBLISHER ===
void VTrackNode::dr_callback(const vilib_ros::tracker_paramConfig& config, const uint32_t& level)
{
    setDRVals(config, false);
}

void VTrackNode::pub_img(const cv_bridge::CvImagePtr& ipt)
{
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr16", ipt->image).toImageMsg();
    imgPub.publish(msg); //Publish image
    ipt->image.release();
}

void VTrackNode::imgCallback(const sensor_msgs::ImageConstPtr& imgp)
{
    try {
        //http://docs.ros.org/kinetic/api/sensor_msgs/html/image__encodings_8h_source.html
        cv_bridge::CvImagePtr imagePtrRaw{ cv_bridge::toCvCopy(imgp, sensor_msgs::image_encodings::BGR16) };
        cv_bridge::CvImagePtr gImg{ cv_bridge::toCvCopy(imgp, sensor_msgs::image_encodings::MONO8) }; //Create tmp gray img for detctor

        //Get image width & height
        int image_width_{ gImg->image.cols };
        int image_height_{ gImg->image.rows };

        std::shared_ptr<vilib::Frame> ff;

        ff = iTracker(gImg, image_width_, image_height_); //Feature detector (FAST) with grayscale img

        processImg(imagePtrRaw, ff, image_width_, image_height_); //Draw the feature point(s)on the img/vid

        pub_img(imagePtrRaw);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr16'.", imgp->encoding.c_str());
    }
}

void VTrackNode::startReconfig()
{
    ft_cb = boost::bind(&VTrackNode::dr_callback, this, _1, _2);
    ft_server.setCallback(ft_cb);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vilib_tracker");
    ros::NodeHandle nh;

    VTrackNode vtn(nh);
    vtn.init();

    ros::spin();

    return 0;
}

