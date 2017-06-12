#include <ros/ros.h>

#include <stdio.h>
#include <string.h>
#include <ctime>
#include <chrono>
#include <iostream>
#include <sstream>
#include <fstream>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;


void LoadImages(const string &strSequence, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);


int main(int argc, char** argv)
{

	if(argc<2)
	{
		cout << "Input argument error" << endl;
	}

    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    LoadImages(string(argv[1]), vstrImageFilenames, vTimestamps);
    int nImages = vstrImageFilenames.size();

	ros::init(argc, argv, "img_pub_node");
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_(nh_);
	image_transport::Publisher image_pub_;
	image_pub_ = it_.advertise("/camera/image_raw", 1);
	
	ros::Rate loop_rate(10);  

	cv_bridge::CvImage cvi;
	cvi.header.frame_id = "image";
	cvi.encoding = "bgr8";

	int cnt = 0;
	while(ros::ok())
	{
		Mat img = imread(vstrImageFilenames[cnt]);
		cout<< cnt << endl;

		if(img.empty())
		{
		    cout << "image read error!" << endl;
		    return 0;
		}
		//imshow("img",img);

		ros::Time time=ros::Time::now();
		
		cvi.header.stamp = time;
		cvi.image = img;

		sensor_msgs::Image im;
		cvi.toImageMsg(im);
		image_pub_.publish(im);
	

		//char c = waitKey(10);
		//if(c=='q')
		//	break;
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

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
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

    const int nTimes = vTimestamps.size();
    vstrImageFilenames.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageFilenames[i] = strPrefixLeft + ss.str() + ".png";
    }
}
