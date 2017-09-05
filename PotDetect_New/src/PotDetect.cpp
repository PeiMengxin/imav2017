/*
 * main.cpp
 *
 *      Author: JYF
 */

#include "PotDetect_New/impl.h"
#include <iostream>
#include <sstream>
#include <vector>
#include <queue>
#include <ros/ros.h>
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include "std_msgs/String.h"

#define FILTER_LEN 2 * LIFE
using namespace std;
using namespace cv;

double max_area;
double min_area;
Logistic log_hog;
Logistic log_hist;
int histFeatLen;
int hogFeatLen;
int pixFeatLen;
int bowFeatLen;

Mat frame;
Mat lastFrame;
Mat filtered_frame;

vector<Target> targets;
vector<Target> valid_targets;

string winName = "pot";
Point pTrackPos;

char first_frame = 1;
int center_x = 0, center_y = 0;
char b_tracking = 0;
int filter_len = FILTER_LEN;
int window_len = 0;
int ksize = 3;
int start = 0;
long c = 0;
bool b_NewFrame;
bool b_FrameFinished;
bool b_Tracking;
bool b_Marking;

ros::Publisher feedback_pub;
ros::Publisher ctrl_pub;

void calcFeatLens()
{
	histFeatLen = 256 / HIST_SEG;
	histFeatLen = histFeatLen * histFeatLen * histFeatLen;

	hogFeatLen = ((HOG_SIZE - BLOCK_SIZE) / STRIDE + 1) * ((HOG_SIZE - BLOCK_SIZE) / STRIDE + 1) * 36;
	pixFeatLen = PIX_SIZE * PIX_SIZE * 3;

	bowFeatLen = 256 / BOW_SEG;
	bowFeatLen = bowFeatLen * bowFeatLen * bowFeatLen * BOW_NUM * BOW_NUM;
}

void string2msg(const char *info, std_msgs::String &msg)
{
	stringstream ss;
	ss << info;
	msg.data = ss.str();
}

void getImageCallback(const sensor_msgs::ImageConstPtr &msg)
{
	try
	{
		frame = cv_bridge::toCvShare(msg, "bgr8")->image;
		if (frame.rows == 0 or frame.cols == 0)
			return;
		b_NewFrame = true;

		Mat dummy;
		frame.copyTo(dummy);

		std_msgs::String feedback_msg;
		if (first_frame)
		{
			first_frame = 0;
			center_x = frame.cols / 2;
			center_y = frame.rows / 2;
			filtered_frame.create(dummy.size(), CV_8UC1);
		}
		if (start)
		{
			if (filter_len > 0)
			{
				detect_bbox(lastFrame, targets, false);
				//if(valid_targets.size()>0){	draw_bbox(dummy, valid_targets, 1);	}
			}
			else
			{
				filter_len = FILTER_LEN + 1;
				filtered_frame.setTo(0);
				detect_bbox(frame, targets, true);
				valid_targets = targets;
				draw_bbox(dummy, targets, 2);
			}
		}
		if (start and filter_len == 0)
		{
			filter_len = FILTER_LEN + 1;
			int max_dist = 0;
			int min_dist = 1e4;
			stringstream ctrl_ss;

			if (b_Tracking)
			{
				std_msgs::String ctrl_msg;
				if (targets.size() == 0)
				{
					string2msg("no-target", ctrl_msg);
					ctrl_pub.publish(ctrl_msg);
				}
				else
				{
					int dx, dy, dist;
					for (size_t i = 0; i < targets.size(); ++i)
					{
						Target &t = targets[i];
						int cur_cx = t.location.x + (t.location.width >> 1);
						int cur_cy = t.location.y + (t.location.height >> 1);
						dist = abs(cur_cx - pTrackPos.x) + abs(cur_cy - pTrackPos.y);
						if (dist < min_dist)
						{
							dx = center_x - cur_cx;
							dy = center_y - cur_cy;
							pTrackPos.x = cur_cx;
							pTrackPos.y = cur_cy;
						}
					}
					cout << dx << "," << dy << endl;
					ctrl_ss << dx << "," << dy << "," << dist;
					string2msg(ctrl_ss.str().c_str(), ctrl_msg);
					ctrl_pub.publish(ctrl_msg);
				}
			}
			else if (not b_Marking)
			{
				for (size_t i = 0; i < targets.size(); ++i)
				{
					Target &t = targets[i];
					int dist = abs(t.location.x - center_x) + abs(t.location.y - center_y);
					if (dist > max_dist)
					{
						max_dist = dist;
						pTrackPos.x = t.location.x + (t.location.width >> 1);
						pTrackPos.y = t.location.y + (t.location.height >> 1);
					}
				}
				//new target found
				string2msg("found", feedback_msg);
				feedback_pub.publish(feedback_msg);
			}
		}
		frame.copyTo(lastFrame);

		string2msg("ok", feedback_msg);
		feedback_pub.publish(feedback_msg);
		//imshow(winName, dummy);
		waitKey(1);
		frame.copyTo(lastFrame);
		if (start)
		{
			--filter_len;
		}
		if (start == 0)
		{
			++start;
		}
		++c;
	}
	catch (cv_bridge::Exception &e)
	{
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}
}

void getInfoCallback(const std_msgs::String::ConstPtr &msg)
{
	const char *data = msg->data.c_str();
	if (strcmp(data, "quit") == 0)
	{
		b_FrameFinished = true;
		cout << "Real frame count: " << c << endl;
		destroyWindow(winName);
		ros::shutdown();
	}
	else if (strcmp(data, "ok") == 0)
	{
		b_FrameFinished = false;
	}
	else if (strcmp(data, "track") == 0)
	{
		b_Tracking = true;
		//b_Marking=false;
		;
	}
	else if (strcmp(data, "marking") == 0)
	{
		b_Tracking = false;
		b_Marking = true;
	}

	//this "search" command informs this node that the next target is required
	//It can be sent by the uav controlling node
	else if (strcmp(data, "search") == 0)
	{
		b_Tracking = false;
		b_Marking = false;
	}
}

int __main(int argc, char **argv)
{
	/*	train_logistic_hist(1e5);*/

	string path = "I:/TestOpenCV/Videos/pot_train/samples/brick.jpg";
	Mat image = imread(path);

	calcFeatLens();
	Logistic logi;
	get_logistic(logi, "hog");
	float res = detect_logistic_hog(image, logi, hogFeatLen);
	cout << "result: " << res << endl;

	return 0;
}

int main(int argc, char **argv)
{
	calcFeatLens();
	get_logistic(log_hist, "hist");
	get_logistic(log_hog, "hog");

	cout << "classifier loaded" << endl;
	//namedWindow(winName);
	startWindowThread();
	ros::init(argc, argv, "PotDetect");
	ros::NodeHandle n_handle;
	image_transport::ImageTransport it(n_handle);
	image_transport::Subscriber image_sub = it.subscribe("imav/video_capture", 1, getImageCallback);
	ros::Subscriber inform_sub = n_handle.subscribe("inform", 500, getInfoCallback);
	feedback_pub = n_handle.advertise<std_msgs::String>("feedback_info", 500);
	ctrl_pub = n_handle.advertise<std_msgs::String>("ctrl_info", 100);
	//ros::Rate loop_rate(5);
	ros::spin();

	return 0;
}
