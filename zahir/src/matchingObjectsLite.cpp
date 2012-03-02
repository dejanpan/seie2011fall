#include <ros/ros.h>
#include <ros/node_handle.h>
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/CvBridge.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <string.h>
#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "features_2d/features_2d.h"
#include <iostream>
#include "matchingObjectsLite.h"

using namespace cv;
using namespace std;
namespace f2d = features_2d;

#define DRAW_RICH_KEYPOINTS_MODE     0
#define DRAW_OUTLIERS_MODE           0

class ImageSubscriber {

public:
	ImageSubscriber(ros::NodeHandle &n) :
		n_(n), it_(n_) {

		image_sub_ = it_.subscribe("/camera/image_raw", 1,
				&ImageSubscriber::imageCallback, this);
		n_.param("save_image", save_image_, std::string(""));
		winName = "correspondences";
		// here the image of the scene
		n_.param("image_name", imageName, std::string(
				"../data/box_in_scene.png"));
		cv_image = NULL;
		n_.param("ransac_projection_threshold", ransacReprojThreshold, 5.0);

	}

	~ImageSubscriber() {

	}

	void drawMatchesOnly(cv::Mat& image1,
			const std::vector<cv::KeyPoint>& keypoints1, cv::Mat& image2,
			const std::vector<cv::KeyPoint>& keypoints2, const std::vector<
					f2d::Match>& matches, cv::Mat& display) {
		// Set up composite image
		//display = cv::Mat::zeros(std::max(image1.rows, image2.rows), image1.cols + image2.cols, CV_8UC3);
		//cv::Mat sub_display1 = display( cv::Rect(0, 0, image1.cols, image1.rows) );
		//cv::Mat sub_display2 = display( cv::Rect(image1.cols, 0, image2.cols, image2.rows) );

		//cv::resize(image1,image1, Size(512,384),0,0,INTER_LINEAR);
		//cv::resize(image2,image2,Size(512,384),0,0,INTER_LINEAR);

		cvtColor(image1, image1, CV_GRAY2BGR);
		cvtColor(image2, image2, CV_GRAY2BGR);

		// Draw lines between matches
		//int shift_bits = 4; //check what they are doing
		//int multiplier = 1 << shift_bits; //check
		printf("Size of the matches: %d\n", matches.size());
		for (std::vector<f2d::Match>::const_iterator i = matches.begin(), ie =
				matches.end(); i != ie; ++i) {
			const cv::KeyPoint& keypt1 = keypoints1[i->index1];
			const cv::KeyPoint& keypt2 = keypoints2[i->index2];
			const Scalar red = CV_RGB(255, 0, 0);
			const Scalar color = CV_RGB(std::rand() % 256, std::rand() % 256,
					std::rand() % 256);
			const int thickness = 8, lineType = 8, radius = 2;
			cv::circle(image1, Point(keypt1.pt.x, keypt1.pt.y), radius, color,
					thickness, lineType);
			printf("Keypoint1 x : %f, y : %f\n", keypt1.pt.x, keypt1.pt.y);
			printf("Keypoint2 x : %f, y : %f\n", keypt2.pt.x, keypt2.pt.y);
			cv::circle(image2, Point(keypt2.pt.x, keypt2.pt.y), radius, color,
					thickness, lineType);
			//cv::line(display,Point(keypt1.pt.x,keypt1.pt.y),Point(keypt2.pt.x,keypt2.pt.y),red, 1, CV_AA, shift_bits);

			//cvShowImage("Image window 2",image2);

			//cv::Point center1(keypt1.pt.x * multiplier, keypt1.pt.y * multiplier); //try resizing image 640*480
			//cv::Point center2((keypt2.pt.x + image1.cols) * multiplier, keypt2.pt.y * multiplier);
			//cv::Scalar color(std::rand() % 256, std::rand() % 256, std::rand() % 256);
			//cv::line(display, center1, center2, color, 1, CV_AA, shift_bits);
		}
		imshow("Window1", image1);
		imshow("Window2", image2);
		waitKey(0);
	}

	void imageCallback(const sensor_msgs::ImageConstPtr& msg_ptr) {

		try {
			cv_image = bridge_.imgMsgToCv(msg_ptr, "bgr8");
		} catch (sensor_msgs::CvBridgeException error) {
			ROS_ERROR("error");
		}

		cout
				<< "< Creating detector, descriptor extractor and descriptor matcher ..."
				<< endl;
		cout << "< Reading the images..." << endl;
		//Mat img1 (cv_image);
		// here the image of the object alone
		Mat img1 = imread("../data/box.png", 0);
		//cvtColor(img1, img1, CV_RGB2GRAY);
		Mat img2 = imread(imageName, 0);
		cout << ">" << endl;
		if (img1.empty() || img2.empty()) {
			cout << "Can not read images" << endl;
			return;
		}

		cout << endl << "< Extracting keypoints from the first image..."
				<< endl;
		vector<KeyPoint> keypoints1;
		detector.detect(img1, keypoints1);
		cout << keypoints1.size() << " points" << endl << ">" << endl;

		cout << "< Computing descriptors for keypoints from the first image..."
				<< endl;
		Mat descriptors1;
		descriptorExtractor.compute(img1, keypoints1, descriptors1);
		cout << ">" << endl;

		cout << endl << "< Extracting keypoints from the second image..."
				<< endl;
		vector<KeyPoint> keypoints2;
		detector.detect(img2, keypoints2);
		cout << keypoints2.size() << " points" << endl << ">" << endl;

		cout
				<< "< Computing descriptors for keypoints from the second image..."
				<< endl;
		Mat descriptors2;
		descriptorExtractor.compute(img2, keypoints2, descriptors2);
		cout << ">" << endl;

		vector<f2d::Match> matches;
		descriptorMatcher.matchWindowless(descriptors1, descriptors2, matches);

		vector<Point2f> points1, points2;
		for (size_t i = 0; i < matches.size(); i++) {
			points1.push_back(keypoints1[matches[i].index1].pt);
			points2.push_back(keypoints2[matches[i].index2].pt);
		}
		Mat H = findHomography(Mat(points1), Mat(points2), CV_RANSAC,
				ransacReprojThreshold);
		Mat points1Projected;
		perspectiveTransform(Mat(points1), points1Projected, H);

		vector<f2d::Match> matchesFiltered;
		for (size_t i = 0; i < points1.size(); i++) {
			Point2f point1 = points1Projected.at<Point2f> (i, 0);
			double dist = norm(point1 - points2[i]);
			if (dist < ransacReprojThreshold) {
				matchesFiltered.push_back(matches[i]);
			}
		}

		//namedWindow(winName, 1);
		Mat drawImg;
		drawMatchesOnly(img1, keypoints1, img2, keypoints2, matchesFiltered,
				drawImg);
		//imshow(winName, drawImg);


		return;
	}

protected:

	ros::NodeHandle n_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	sensor_msgs::CvBridge bridge_;
	std::string save_image_;
	string winName;
	string imageName;
	IplImage *cv_image;
	double ransacReprojThreshold;
	cv::SiftFeatureDetector detector;
	cv::SiftDescriptorExtractor descriptorExtractor;
	f2d::BruteForceMatcher<f2d::L2<float> > descriptorMatcher;

};

int main(int argc, char** argv) {
	ros::init(argc, argv, "matchingObjects");
	ros::NodeHandle n("~");
	ImageSubscriber ic(n);
	ros::spin();
	return 0;
}

matchingObjectsLite::matchingObjectsLite() {
	// TODO Auto-generated constructor stub

}

matchingObjectsLite::~matchingObjectsLite() {
	// TODO Auto-generated destructor stub
}
