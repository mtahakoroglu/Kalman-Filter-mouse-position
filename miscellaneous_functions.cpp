#include "miscellaneous_functions.h"

vector<int> set_image_write_quality(const char* suffix, const int quality)
{
	vector<int> compressionParameters;
	if (suffix == "jpg")
	{
		compressionParameters.push_back(IMWRITE_JPEG_QUALITY);
		compressionParameters.push_back(quality);
	}
	else if (suffix == "png")
	{
		compressionParameters.push_back(IMWRITE_PNG_COMPRESSION);
		compressionParameters.push_back(quality);
	}
	else
	{
		cerr << "Unknown suffix for writing image!" << endl;
		system("pause");
		exit(EXIT_FAILURE);
	}
	return compressionParameters;
}

void open_file_stream_to_read(ifstream& fin, const string& filename)
{
	fin.open(filename, ios::in);
	if (!fin.is_open())
	{
		cerr << "Error in reading from " << filename << " file!" << endl;
		exit(EXIT_FAILURE);
	}
}

void open_file_stream_to_write(ofstream& fout, const string& filename)
{
	fout.open(filename, ios::out);
	if (!fout.is_open())
	{
		cerr << "Error in opening " << filename << " to write!" << endl;
		exit(EXIT_FAILURE);
	}
}

void display_OpenCV_version()
{
	cout << "Compiled with OpenCV version " << CV_VERSION << endl; // thnx to Shervin Emami for this label.
}

void set_kalman_filter_parameters(KalmanFilter& KF, const int n, const int m, const POINT& initialState, const float observationNoiseSTDdeviation)
{
	/* MotionModel = 'ConstantVelocity' */
	// state transition matrix A. size of A is 6 x 6 for 'ConstantAcceleration', 4 x 4 for 'ConstantVelocity'
	// following matrix is the StateTransitionModel in MATLAB CVT
	// x = (x, u, y, v)'
	KF.transitionMatrix = (Mat_<float>(n, n) << 1.0, 0.0, 1.0, 0.0,
												0.0, 1.0, 0.0, 1.0,
												0.0, 0.0, 1.0, 0.0,
												0.0, 0.0, 0.0, 1.0);
	Mat A = KF.transitionMatrix.clone();
	// process noise covariance Q. size of Q is n x n while size of w is n x 1.
	setIdentity(KF.processNoiseCov, Scalar::all(1e-4));
	Mat Q = KF.processNoiseCov.clone();
	// measurement matrix H
	KF.measurementMatrix = (Mat_<float>(m, n) << 1.0, 0.0, 0.0, 0.0,
												 0.0, 1.0, 0.0, 0.0);
	Mat H = KF.measurementMatrix.clone();
	// measurement noise covariance R. size of R is m x m while size of v is m x 1.
	setIdentity( KF.measurementNoiseCov, Scalar::all( pow(observationNoiseSTDdeviation, 2.0) ) );
	Mat R = KF.measurementNoiseCov.clone();
	/* initial state and error covariance estimates. */
	/* remember that posteriors of previous cycle become priors of current cycle when time update (i.e., prediction) is occurring. */
	/* to be more explicit; x_kMinus1Hat (KF.statePre) becomes x_kHat (KF.statePost) and P_kMinus1 (KF.errorCovPre) becomes P_k (KF.errorCovPost) */
	/* these two variables are what we know from our past. We will predict state and error covariance based on these two variables. */
	float temp[4] = { initialState.x, initialState.y, 0.0, 0.0 }; // I manually entered initial state
	Mat x_kminus1Hat = Mat(n, 1, CV_32F, temp);
	KF.statePost = x_kminus1Hat.clone(); //(Mat_<float>(n, 1) << x_kminus1Hat);
	// Mat P_kminus1 = Mat::eye(n, n, CV_32F) * 100000;
	// KF.errorCovPost = P_kminus1.clone(); //(Mat_<float>(n, n) << P_kminus1);
	setIdentity(KF.errorCovPost, Scalar::all(.1));
	Mat P_kminus1 = KF.errorCovPost.clone();
	return;
}

void visualize_trajectory(Mat img, Point2f* trajectory, const int lastNframes, const size_t i, const Point2f state, const Scalar trajectoryColor)
{
	if (i == 1) // first frame in which KF made an estimate
		fill_n(trajectory, lastNframes, state);
	else
	{
		for (int j = lastNframes - 1; j > 0; j--)
			trajectory[j] = trajectory[j - 1];
		*trajectory = state;
		for (int j = 0; j < lastNframes - 1; j++)
			line(img, trajectory[j], trajectory[j + 1], trajectoryColor, 1);
	}
	return;
}

void save_results(const Mat img, const size_t i, const bool saveResultsJPEG)
{
	static vector<int> compressionParameters = set_image_write_quality("jpg", 100);
	if (saveResultsJPEG)
	{
		char outputName[50] = { '\0' };
		sprintf(outputName, "results_images/img%04d.jpg", i);
		imwrite(outputName, img, compressionParameters);
	}
}

Mat add_observation_noise(const POINT& actualMousePosition, const float stdDeviation)
{
	static RNG rng(getTickCount());
	Mat measurement(2, 1, CV_32F);
	Mat v(2, 1, CV_32F); // measurement noise
	rng.fill(v, RNG::NORMAL, 0, stdDeviation);
	measurement.at<float>(0, 0) = actualMousePosition.x + v.at<float>(0, 0);
	measurement.at<float>(1, 0) = actualMousePosition.y + v.at<float>(1, 0);
	return measurement;
}

void draw_trajectory(Mat img, const vector<Point2f>& trajectory, const Scalar& color)
{
	for (int i = 0; i < trajectory.size() - 1; i++)
		line(img, trajectory[i], trajectory[i + 1], color, 1);
}