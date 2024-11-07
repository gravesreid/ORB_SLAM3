#include <signal.h>
#include <stdlib.h>
#include <iostream>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <System.h>

using namespace std;

bool b_continue_session = true;

void exit_loop_handler(int s) {
    cout << "Finishing session" << endl;
    b_continue_session = false;
}

int main(int argc, char **argv) {
    // print out b_continue_session
    cout << "b_continue_session: " << b_continue_session << endl;
    if (argc < 3 || argc > 4) {
        cerr << endl
             << "Usage: ./mono_url_video path_to_vocabulary path_to_settings (trajectory_file_name)"
             << endl;
        return 1;
    }

    string file_name;
    if (argc == 4) {
        file_name = string(argv[argc - 1]);
    }

    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = exit_loop_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);

    // Set up OpenCV to capture from the video stream URL
    string stream_url = "http://192.168.1.44:8000/stream.mjpg";
    cv::VideoCapture cap(stream_url);
    if (!cap.isOpened()) {
        cerr << "Error: Could not open video stream from URL: " << stream_url << endl;
        return -1;
    }

    // Create ORB-SLAM3 system
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, true, 0, file_name);
    float imageScale = SLAM.GetImageScale();

    cv::Mat frame, resized_frame;
    double timestamp;
    double t_resize = 0.f, t_track = 0.f;

    while (b_continue_session) {
        auto start = chrono::steady_clock::now();

        // Capture a frame
        if (!cap.read(frame)) {
            cerr << "Error: Failed to capture frame from video stream" << endl;
            break;
        } else {
            cout << "Frame captured successfully" << endl;
        }


        // Resize the image if necessary
        if (imageScale != 1.f) {
            int width = frame.cols * imageScale;
            int height = frame.rows * imageScale;
            cv::resize(frame, resized_frame, cv::Size(width, height));
        } else {
            resized_frame = frame;
        }

        // Pass the frame to ORB-SLAM3
        auto now = chrono::steady_clock::now();
        double timestamp = chrono::duration_cast<chrono::duration<double>>(now.time_since_epoch()).count();
 
        SLAM.TrackMonocular(frame, timestamp);

        // Check for exit condition
        if (cv::waitKey(1) == 27) { // Press 'ESC' to exit
            break;
        }
    }

    // Release resources
    cap.release();
    SLAM.Shutdown();
    cout << "System shutdown!" << endl;
    return 0;
}
