/*
  Project: kalman_filter_mouse_example
  Written by & on: Michael Silverhouse, 6-7-16
  Synopsis: This is a very good example to understand Kalman Filter. The problem is bivariate; we estimate mouse location in x and y directions.
  This code worked nicely with OpenCV 3.1.0 C++ libraries in Visual Studio 2015 IDE.
  Note: I borrowed this example from 
  http://opencvexamples.blogspot.com/2014/01/kalman-filter-implementation-tracking.html
  I made some changes to make it simpler, more organized in my opinion. I am not a good programmer, you can make it much better.
*/

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp" // KalmanFilter class
#include "miscellaneous_functions.h"

int main(int argc, char** argv)
{
	/* n = 4 shows that we are using "constant velocity model". if n = 6 then it means our motion model is "constant acceleration". */
	const int n = 4, m = 2; // n is dimensionality of state vector x_kHat, m is dimensionality of measurement vector z_k
	KalmanFilter KF(n, m, 0, CV_32F); // instantiate Kalman filter, n is state dimensionality, m is measurement dimensionality
	/* mouse position in the screen is our measurements in this problem. Of course I will add some noise to actual position. */
	POINT mouse;
	GetCursorPos(&mouse);
	const float observationNoiseSTDdeviation = 1; // increase this to see wild jumping observations
	set_kalman_filter_parameters(KF, n, m, mouse, observationNoiseSTDdeviation);
	
	vector<Point2f> measurementTrajectory, KFtrajectory, GTtrajectory;
	measurementTrajectory.clear(); KFtrajectory.clear(); GTtrajectory.clear();

	display_OpenCV_version();
	const bool saveResultsJPEG = true;

	int i = 0;
	while (true)
	{
		KF.predict(); // time update
		GetCursorPos(&mouse); /* receive the measurement --> mouse contains ground truth data, i.e., actual position of mouse */
		Mat measurement = add_observation_noise(mouse, observationNoiseSTDdeviation);
		Mat KFestimate = KF.correct(measurement); // measurement update
		/*=========================== visualize all results ======================================*/
		GTtrajectory.push_back( Point2f(mouse.x, mouse.y) );
		measurementTrajectory.push_back( Point2f( measurement.at<float>(0, 0), measurement.at<float>(1, 0) ) );
		KFtrajectory.push_back( Point2f( KFestimate.at<float>(0, 0), KFestimate.at<float>(1, 0) ) );
		Mat img(600, 800, CV_8UC3, Scalar(0)); // initialize a black image
		drawMarker(img, Point( mouse.x, mouse.y ), GREEN_COLOR, MARKER_TILTED_CROSS, 10, 2, LINE_8);
		drawMarker(img, Point( KFestimate.at<float>(0, 0), KFestimate.at<float>(1, 0) ), YELLOW_COLOR, MARKER_TILTED_CROSS, 10, 2, LINE_8);
		//drawMarker(img, Point( measurement.at<float>(0, 0), measurement.at<float>(1, 0) ), RED_COLOR, MARKER_TILTED_CROSS, 10, 2, LINE_8);
		draw_trajectory(img, GTtrajectory, GREEN_COLOR);
		draw_trajectory(img, KFtrajectory, YELLOW_COLOR);
		//draw_trajectory(img, measurementTrajectory, RED_COLOR);
		save_results(img, i, saveResultsJPEG);
		imshow("Kalman Filter example - mouse position", img);
		i++;
		if (waitKey(10) == 27)
		{
			destroyAllWindows();
			break;
		}
	}

	return 0;
}