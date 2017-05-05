// kinectTest.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"

#define _SCL_SECURE_NO_WARNINGS
#define _CRT_SERCURE_NO_WARNINGS
#include <pcl/io/openni2_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/filters/passthrough.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <direct.h>
//#include <pcl/visualization/pcl_visualizer.h>

using namespace std;
using namespace pcl;
using namespace cv;

PointCloud<PointXYZRGBA>::Ptr cloudptr(new PointCloud<PointXYZRGBA>); // A cloud that will store color info.
PointCloud<PointXYZ>::Ptr fallbackCloud(new PointCloud<PointXYZ>); // A fallback cloud with just depth data.
PointCloud<PointXYZRGBA>::Ptr filteredCloud(new PointCloud<PointXYZRGBA>);
boost::shared_ptr<visualization::CloudViewer> viewer;                 // Point cloud viewer object.
																	  //boost::shared_ptr<visualization::PCLVisualizer> viewer;

Grabber* openniGrabber;                                               // OpenNI grabber that takes data from the device.
unsigned int fileCount = 0;                                          // For the numbering of the clouds saved to disk.
bool saveCloud(false), noColor(false);                                // Program control.
bool exitProg(false);
//stringstream stream;
//string fileName;
std::string fileName;
std::string fullPath1;
std::string fullPath2;
std::string fullPath3;
std::string fullPath4;
//make directory
void specifyFileName(std::string &saveFileName) {
	char key = 0;
	while (key != 'y') {
		std::cout << "Specify the output file name:" << std::endl;
		std::cin >> saveFileName;
		std::cout << "Confirm?(y or n)" << std::endl;
		std::cin >> key;
	}
}

void GetFilePath(std::string &fileName, std::string &fullPath1, std::string &fullPath2, std::string &fullPath3, std::string &fullPath4)
{
	string strPath1;
	string strPath2;
	string strPath3;
	string strPath4;

	int nRes = 0;

	const string subDir_1("data\\");
	string subDir_2 = fileName;
	const string subDir_31("\\image\\");
	const string subDir_32("\\cloud\\");
	const string subDir_41("color\\");
	const string subDir_42("depth\\");
	const string subDir_43("pcd\\");
	const string subDir_44("ply\\");

	string subDir_12 = subDir_1 + subDir_2;

	//指定路径            

	strPath1 = subDir_12 + subDir_31 + subDir_41;
	strPath2 = subDir_12 + subDir_31 + subDir_42;
	strPath3 = subDir_12 + subDir_32 + subDir_43;
	strPath4 = subDir_12 + subDir_32 + subDir_44;
	namespace fs = boost::filesystem;

	//路径的可移植
	fs::path full_path1(fs::initial_path());
	fs::path full_path2(fs::initial_path());
	fs::path full_path3(fs::initial_path());
	fs::path full_path4(fs::initial_path());
	full_path1 = fs::system_complete(fs::path(strPath1, fs::native));
	full_path2 = fs::system_complete(fs::path(strPath2, fs::native));
	full_path3 = fs::system_complete(fs::path(strPath3, fs::native));
	full_path4 = fs::system_complete(fs::path(strPath4, fs::native));
	//判断二级子目录是否存在，不存在则需要创建
	if (!fs::exists(subDir_12))
	{
		fs::create_directories(full_path1);
		fs::create_directories(full_path2);
		fs::create_directories(full_path3);
		fs::create_directories(full_path4);
	}
	fullPath1 = full_path1.generic_string();
	fullPath2 = full_path2.generic_string();
	fullPath3 = full_path3.generic_string();
	fullPath4 = full_path4.generic_string();

}

// This function is called every time the device has new data.
void grabberCallback(const PointCloud<PointXYZRGBA>::ConstPtr& cloud) {
	if (!viewer->wasStopped())
		viewer->showCloud(cloud);
	//viewer->addPointCloud(cloud);

	if (saveCloud)
	{
		std::cout << "save start: don't touch keyboard!!!!" << std::endl;
		stringstream stream;
		string savefileName;
		stream.str("");
		stream << fileName << "_" << fileCount << ".pcd";
		savefileName = fullPath3 + stream.str();
		io::savePCDFileASCII(savefileName, *cloud);
		stream.str("");
		stream << fileName << "_" << fileCount << ".ply";
		savefileName = fullPath4 + stream.str();
		io::savePLYFileASCII(savefileName, *cloud);
		// Filter object.
		//		PassThrough<pcl::PointXYZRGBA> filter;
		//		filter.setInputCloud(cloud);
		// Filter out all points with Z values not in the [0-2] range.
		//		filter.setFilterFieldName("z");
		//		filter.setFilterLimits(0.0, 1.5);

		//		filter.filter(*filteredCloud);
		//		if ((io::savePLYFileASCII(filename, *cloud) == 0) && ())
		//		{
		fileCount++;
		//			cout << "Saved " << filename << "." << endl;
		//		}
		//		else PCL_ERROR("Problem saving %s.\n", filename.c_str());

		saveCloud = false;
		std::cout << "save done!" << std::endl;
	}
}

// For detecting when SPACE is pressed.
void keyboardEventOccurred(const visualization::KeyboardEvent& event, void* nothing)
{
	if (event.getKeySym() == "space" && event.keyDown())
		saveCloud = true;
	else if (event.getKeyCode() == 27 && event.keyDown())
		exitProg = true;
}

// Creates, initializes and returns a new viewer.
boost::shared_ptr<visualization::CloudViewer> createViewer() {
	boost::shared_ptr<visualization::CloudViewer> v
	(new visualization::CloudViewer("OpenNI viewer"));
	v->registerKeyboardCallback(keyboardEventOccurred);

	return (v);
}
/*
boost::shared_ptr<visualization::PCLVisualizer> createViewer() {
boost::shared_ptr<visualization::PCLVisualizer> v
(new visualization::PCLVisualizer("OpenNI viewer"));
v->registerKeyboardCallback(keyboardEventOccurred);

return (v);
}*/

int main(int argc, char** argv)
{

	if (openni::OpenNI::initialize() != openni::STATUS_OK) {
		cerr << "Error:	" << openni::OpenNI::getExtendedError() << endl;
	}



	//创建目录


	specifyFileName(fileName);
	GetFilePath(fileName, fullPath1, fullPath2, fullPath3, fullPath4);

	//捕捉点云
	openniGrabber = new pcl::io::OpenNI2Grabber();
	if (openniGrabber == 0)
		return -1;
	boost::function<void(const PointCloud<PointXYZRGBA>::ConstPtr&)> f =
		boost::bind(&grabberCallback, _1);
	openniGrabber->registerCallback(f);
	viewer = createViewer();
	//viewer->setCameraPosition(-1, 0, 0, 0, 0, 0, 0, 0, 1);
	openniGrabber->start();

	openni::VideoStream oniDepthStream;
	openni::VideoStream oniColorStream;
	openni::VideoMode modeDepth;
	openni::VideoMode modeColor;


	//		std::cout << "checkponit1" << std::endl;
	//捕捉彩色图和深度图
	openni::Device device;
	device.open(openni::ANY_DEVICE);
	oniDepthStream.create(device, openni::SENSOR_DEPTH);
	modeDepth.setResolution(640, 480);
	modeDepth.setFps(30);
	modeDepth.setPixelFormat(openni::PIXEL_FORMAT_DEPTH_1_MM);
	oniDepthStream.setVideoMode(modeDepth);
	oniDepthStream.start();

	oniColorStream.create(device, openni::SENSOR_COLOR);
	modeColor.setResolution(640, 480);
	modeColor.setFps(30);
	modeColor.setPixelFormat(openni::PIXEL_FORMAT_RGB888);
	oniColorStream.setVideoMode(modeColor);
	oniColorStream.start();

	openni::VideoFrameRef oniDepthImg;
	openni::VideoFrameRef oniColorImg;

	cv::Mat cvDepthImg;
	cv::Mat cvBGRImg;

	//	int ImgNum = 0;
	//	char saveFileName[50];

	cv::namedWindow("depth");
	cv::namedWindow("color");



	// Main loop.
	//	while (!viewer->wasStopped()) {
	while (!exitProg) {
		if (oniColorStream.readFrame(&oniColorImg) == openni::STATUS_OK)
		{
			cv::Mat cvRGBImg(oniColorImg.getHeight(), oniColorImg.getWidth(), CV_8UC3, (void*)oniColorImg.getData());
			cv::cvtColor(cvRGBImg, cvBGRImg, CV_RGB2BGR);
			cv::imshow("color", cvBGRImg);
		}

		if (oniDepthStream.readFrame(&oniDepthImg) == openni::STATUS_OK)
		{
			cv::Mat cvRawImg16U(oniDepthImg.getHeight(), oniDepthImg.getWidth(), CV_16UC1, (void*)oniDepthImg.getData());
			cvRawImg16U.convertTo(cvDepthImg, CV_8U, 255.0 / (oniDepthStream.getMaxPixelValue()));
			cv::imshow("depth", cvDepthImg);
		}

		cv::waitKey(10);

		if (saveCloud) {
			stringstream stream;
			string savefileName;
			stream.str();
			stream << fileName << "_" << fileCount << ".png";
			savefileName = fullPath1 + stream.str();
			cv::imwrite(savefileName, cvBGRImg);
			stream.str("");
			stream << fileName << "_" << fileCount << ".png";
			savefileName = fullPath2 + stream.str();
			cv::imwrite(savefileName, cvDepthImg);
		}

		boost::this_thread::sleep(boost::posix_time::milliseconds(20));
	}

	openniGrabber->stop();
	return 0;

}