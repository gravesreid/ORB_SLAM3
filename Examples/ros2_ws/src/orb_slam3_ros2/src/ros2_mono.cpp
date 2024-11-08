/**
* This file is part of ORB-SLAM3
* Adapted for ROS2 by: Reid Graves (Carnegie Mellon University)
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"

#include "../../../../include/System.h"

using namespace std;

class VideoStreamGrabber : public rclcpp::Node
{
public:
    VideoStreamGrabber(ORB_SLAM3::System* pSLAM, const std::string& stream_url)
        : Node("video_stream_grabber"), mpSLAM(pSLAM), stream_url_(stream_url)
    {
        // Set up OpenCV to capture from the video stream URL
        cap_.open(stream_url_);
        if (!cap_.isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "Error: Could not open video stream from URL: %s", stream_url_.c_str());
            rclcpp::shutdown();
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Successfully opened video stream from URL: %s", stream_url_.c_str());
        }

        // Start the timer to grab frames
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(30),
            std::bind(&VideoStreamGrabber::GrabFrame, this));
    }

private:
    void GrabFrame()
    {
        cv::Mat frame;
        if (!cap_.read(frame))
        {
            RCLCPP_ERROR(this->get_logger(), "Error: Failed to capture frame from video stream");
            return;
        }

        // Convert cv::Mat to ROS2 sensor_msgs::msg::Image
        auto now = this->now();
        std_msgs::msg::Header header;
        header.stamp = now;
        header.frame_id = "camera";

        cv_bridge::CvImage cv_image(header, "bgr8", frame);
        sensor_msgs::msg::Image::SharedPtr msg = cv_image.toImageMsg();

        // Pass the frame to ORB-SLAM3
        mpSLAM->TrackMonocular(cv_image.image, now.seconds());
    }

    ORB_SLAM3::System* mpSLAM;
    std::string stream_url_;
    cv::VideoCapture cap_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    if (argc != 4)
    {
        std::cerr << std::endl << "Usage: ros2 run ORB_SLAM3 Mono path_to_vocabulary path_to_settings stream_url" << std::endl;
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, true);

    std::string stream_url = argv[3];

    auto node = std::make_shared<VideoStreamGrabber>(&SLAM, stream_url);

    rclcpp::spin(node);

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    rclcpp::shutdown();

    return 0;
}

