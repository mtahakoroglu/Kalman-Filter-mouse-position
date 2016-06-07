#pragma once
#include "opencv2/imgcodecs/imgcodecs.hpp" // imread(), imwrite()
#include "opencv2/video/tracking.hpp" // KalmanFilter class
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream> // cout()
#include <fstream>
#include <Windows.h> // POINT data structure, GetCursorPos()

#define RED_COLOR Scalar(0, 0, 255)
#define GREEN_COLOR Scalar(0, 255, 0)
#define YELLOW_COLOR Scalar(0, 255, 255)
#define WHITE_COLOR Scalar(255, 255, 255)
#define BLUE_COLOR Scalar(255, 0, 0)

using namespace cv;
using namespace std;

/* for jpg --> pass [0-100] quality values, for png --> [0-9] compression parameters */
vector<int> set_image_write_quality(const char* suffix, const int quality);
void open_file_stream_to_read(ifstream& fin, const string& filename);
void open_file_stream_to_write(ofstream& fout, const string& filename);
void display_OpenCV_version();
void set_kalman_filter_parameters(KalmanFilter& KF, const int n, const int m, const POINT& initialState, const float observationNoiseSTDdeviation);
void visualize_trajectory(Mat img, Point2f* trajectory, const int lastNframes, const size_t i, const Point2f initialState, const Scalar trajectoryColor);
void save_results(const Mat img, const size_t i, const bool saveResultsJPEG);
Mat add_observation_noise(const POINT& actualMousePosition, const float stdDeviation);
void draw_trajectory(Mat img, const vector<Point2f>& measurementTrajectory, const Scalar& color);
void save_results(const Mat img, const size_t i, const bool saveResultsJPEG);