#include <ros/ros.h>

#include <stdio.h>
#include <string.h>
#include <ctime>
#include <chrono>
#include <iostream>
#include <sstream>
#include <fstream>
#include <thread>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

Mat imgLeft;
Mat imgRight;	
Mat imgLeftDist, imgRightDist;

void LoadImages(const string &strPathLeft, const string &strPathRight, const string &strPathTimes,
                vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimeStamps);


void imgPub(const image_transport::Publisher pub, const sensor_msgs::Image img)
{
	pub.publish(img);
}

void imageGrab(const string dir, const int side)
{
	if(side == 0)
		imgLeftDist = cv::imread(dir);
	if(side == 1)
		imgRightDist = cv::imread(dir);
}

int main(int argc, char** argv)
{
	if(argc<2)
	{
		cout << "Input argument error:" << endl;
		cout << "rosrun dataset_publisher stereo_EuRoc_publisher seq_addr" << endl;
                	return -1;
	}

	string path_to_seq = string(argv[1]) ;
	// Retrieve paths to images
	vector<string> vstrImageLeft;
	vector<string> vstrImageRight;
	vector<double> vTimeStamp;

	string path_to_left_folder = path_to_seq + "/mav0/cam0/data";
	string path_to_right_folder = path_to_seq + "/mav0/cam1/data";
	string path_to_times_file = path_to_seq + "/times.txt";
	LoadImages(path_to_left_folder, path_to_right_folder, path_to_times_file, vstrImageLeft, vstrImageRight, vTimeStamp);

	if(vstrImageLeft.empty() || vstrImageRight.empty())
	{
	    cerr << "ERROR: No images in provided path." << endl;
	    return 1;
	}

	if(vstrImageLeft.size()!=vstrImageRight.size())
	{
	    cerr << "ERROR: Different number of left and right images." << endl;
	    return 1;
	}

	string config_dir = path_to_seq + "/../EuRoC.yaml";
	// Read rectification parameters
	cv::FileStorage fsSettings(config_dir.c_str(), cv::FileStorage::READ);
	if(!fsSettings.isOpened())
	{
	    cerr << "ERROR: Wrong path to settings" << endl;
	    return -1;
	}
	cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
	fsSettings["LEFT.K"] >> K_l;
	fsSettings["RIGHT.K"] >> K_r;

	fsSettings["LEFT.P"] >> P_l;
	fsSettings["RIGHT.P"] >> P_r;

	fsSettings["LEFT.R"] >> R_l;
	fsSettings["RIGHT.R"] >> R_r;

	fsSettings["LEFT.D"] >> D_l;
	fsSettings["RIGHT.D"] >> D_r;

	int rows_l = fsSettings["LEFT.height"];
	int cols_l = fsSettings["LEFT.width"];
	int rows_r = fsSettings["RIGHT.height"];
	int cols_r = fsSettings["RIGHT.width"];

	if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
	        rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
	{
	    cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
	    return -1;
	}

	cv::Mat M1l,M2l,M1r,M2r;
	cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,M1l,M2l);
	cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,M1r,M2r);


	const int nImages = vstrImageLeft.size();

	ros::init(argc, argv, "img_pub_node");
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_(nh_);
	
	image_transport::Publisher image_pub_left;
	image_transport::Publisher image_pub_right;

	image_pub_left = it_.advertise("/camera/left/image_raw", 1);
	image_pub_right = it_.advertise("/camera/right/image_raw", 1);
    
	ros::Rate loop_rate(20);  

	cv_bridge::CvImage cvi_l;
    	cv_bridge::CvImage cvi_r;
	cvi_l.header.frame_id = "image";
    	cvi_r.header.frame_id = "image";
	cvi_l.encoding = "bgr8";
    	cvi_r.encoding = "bgr8";

	int cnt = 0;

	while(ros::ok())
	{
        		std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

		//Mat imgLeft = cv::imread(vstrImageLeft[cnt]);
		//Mat imgRight = cv::imread(vstrImageRight[cnt]);	
		thread leftGrab(imageGrab,vstrImageLeft[cnt],0);
		thread rightGrab(imageGrab,vstrImageRight[cnt],1);
		leftGrab.join();
		rightGrab.join();

  	       	if(imgLeftDist.empty())
	        	{
	            		cerr << endl << "Failed to load image at: "
	             		    << string(vstrImageLeft[cnt]) << endl;
	            		return 1;
	       	}

		cv::remap(imgLeftDist,imgLeft,M1l,M2l,cv::INTER_LINEAR);
		cv::remap(imgRightDist,imgRight,M1r,M2r,cv::INTER_LINEAR);	       	

        		std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

		double timread= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
		if(cnt % 5 == 0)
		{	
			cout << cnt << "  Image reading time = " << timread << "s, freqency = " << 1/timread << "Hz" << endl;
			//cout<< cnt << endl;
	       	}
        		//cout<< cnt << endl;
//		Mat img;
//		cv::resize(imgLeft,img,Size(imgLeft.cols/2,imgLeft.rows/2));
//		imshow("img",img);
//		waitKey(10);

		ros::Time time=ros::Time::now();
		
		cvi_l.header.stamp = time;
        		cvi_r.header.stamp = time;

		cvi_l.image = imgLeft;
        		cvi_r.image = imgRight;

		sensor_msgs::Image im_l;
		sensor_msgs::Image im_r;
		cvi_l.toImageMsg(im_l);
        		cvi_r.toImageMsg(im_r);
        
        		std::thread leftPub(imgPub,image_pub_left,im_l);
        		std::thread rightPub(imgPub,image_pub_right,im_r);
        		leftPub.join();
        		rightPub.join();

		//image_pub_left.publish(im_l);
	    	//image_pub_right.publish(im_r);
    
		// char c = waitKey(10);
		// if(c=='q')
		// 	break;
    		cnt++;
    		if(cnt>=nImages)
    			cnt=0;
    		//break;

		ros::spinOnce();  
    		loop_rate.sleep();  
	}


	//ros::spin();
	return 0;

}

void LoadImages(const string &strPathLeft, const string &strPathRight, const string &strPathTimes,
                vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimeStamps)
{
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);
    vstrImageLeft.reserve(5000);
    vstrImageRight.reserve(5000);
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            vstrImageLeft.push_back(strPathLeft + "/" + ss.str() + ".png");
            vstrImageRight.push_back(strPathRight + "/" + ss.str() + ".png");
            double t;
            ss >> t;
            vTimeStamps.push_back(t/1e9);

        }
    }
}
