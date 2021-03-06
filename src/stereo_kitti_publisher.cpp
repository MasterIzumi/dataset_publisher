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

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps);

void imgPub(const image_transport::Publisher pub, const sensor_msgs::Image img)
{
	pub.publish(img);
}

void imageGrab(const string dir, const int side)
{
	if(side == 0)
		imgLeft = cv::imread(dir);
	if(side == 1)
		imgRight = cv::imread(dir);
}

int main(int argc, char** argv)
{
	if(argc<2)
	{
		cout << "Input argument error:" << endl;
		cout << "rosrun dataset_publisher stereo_kitti_publisher seq_addr" << endl;
                return -1;
	}

	// Retrieve paths to images
	vector<string> vstrImageLeft;
	vector<string> vstrImageRight;
	vector<double> vTimestamps;
	LoadImages(string(argv[1]), vstrImageLeft, vstrImageRight, vTimestamps);
	const int nImages = vstrImageLeft.size();

	ros::init(argc, argv, "img_pub_node");
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_(nh_);
	
	image_transport::Publisher image_pub_left;
	image_transport::Publisher image_pub_right;

	image_pub_left = it_.advertise("/camera/left/image_raw", 1);
	image_pub_right = it_.advertise("/camera/right/image_raw", 1);
    
	ros::Rate loop_rate(5);  

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

  	       	if(imgLeft.empty())
	        	{
	            		cerr << endl << "Failed to load image at: "
	             		    << string(vstrImageLeft[cnt]) << endl;
	            		return 1;
	       	}

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

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }

    string strPrefixLeft = strPathToSequence + "/image_2/";
    string strPrefixRight = strPathToSequence + "/image_3/";

    const int nTimes = vTimestamps.size();
    vstrImageLeft.resize(nTimes);
    vstrImageRight.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageLeft[i] = strPrefixLeft + ss.str() + ".png";
        vstrImageRight[i] = strPrefixRight + ss.str() + ".png";
    }
}
