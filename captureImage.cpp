#include <iostream>
#include <stdio.h>
#include <fstream>
#include <sys/stat.h> 
#include <sys/types.h>
#include <sys/time.h>
#include<ctime>
#include <iomanip>
#include <time.h>
#include <signal.h>
#include "opencv/highgui.h" 
#include "opencv2/imgproc/imgproc_c.h" 
#include "opencv2/core/core.hpp" 
#include "opencv2/imgproc/imgproc.hpp" 
#include "opencv2/highgui/highgui.hpp" 
#include "opencv2/calib3d/calib3d.hpp"
#include <string>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

#ifdef _MKDIR_LINUX
#include <io.h>
#include <direct.h> 
#else
#include <unistd.h>
#include <sys/stat.h>
#endif
#include <stdint.h>

#define MAX_PATH_LEN 256

#ifdef _MKDIR_LINUX
#define ACCESS(fileName,accessMode) _access(fileName,accessMode)
#define MKDIR(path) _mkdir(path)
#else
#define ACCESS(fileName,accessMode) access(fileName,accessMode)
#define MKDIR(path) mkdir(path,S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH)
#endif

using namespace std;
using namespace cv;

enum
{
   Processor_cl,
   Processor_gl,
   Processor_cpu
};

bool protonect_shutdown = false; // Whether the running application should shut down.

void sigint_handler(int s)
{
   protonect_shutdown = true;
}

// 从左到右依次判断文件夹是否存在,不存在就创建
// example: /home/root/mkdir/1/2/3/4/
// 注意:最后一个如果是文件夹的话,需要加上 '\' 或者 '/'
int32_t createDirectory(const std::string &directoryPath)
{
    cout<<"\n开始创建文件夹\n";
    uint32_t dirPathLen = directoryPath.length();
    if (dirPathLen > MAX_PATH_LEN)
    {
        return -1;
    }
    char tmpDirPath[MAX_PATH_LEN] = { 0 };
    for (uint32_t i = 0; i < dirPathLen; ++i)
    {
        tmpDirPath[i] = directoryPath[i];
        if (tmpDirPath[i] == '\\' || tmpDirPath[i] == '/')
        {
            if (ACCESS(tmpDirPath, 0) != 0)
            {
                int32_t ret = MKDIR(tmpDirPath);
                if (ret != 0)
                {
                    return ret;
                }
            }
        }
    }
    return 0;
}

// void drawText(Mat & image, const char* a );



int main(int argc, char** argv)
{
	//定义变量
	double minVal = 0, maxVal = 0;
	struct timeval tv;
	long Time0 = 0; 
	long Time1 = 0; 
	int Time11 = 0; 
	int Time22 = 0; 
	double Sum = 0;
	gettimeofday(&tv, NULL);
	time_t t1;
	time_t t2;
	time(&t1);
	double NotStartStream1 = 0; //设备1不能打开流的次数 
	double NotStartStream2 = 0; //设备2不能打开流的次数 
	double WaitFailed1 = 0; //设备1等待超时的次数 
	double WaitFailed2 = 0; //设备1等待超时的次数
	double Time_frame = 0;
	bool SHOW_ON = false;//默认关闭显示
	
	std::cout << "Hello World!" << std::endl;
	libfreenect2::Freenect2 freenect2;
	libfreenect2::Freenect2Device *dev = 0;
	libfreenect2::PacketPipeline  *pipeline = 0;
	
	cv::Mat frameResized;
	
	char status_show[10];
	std::cout << "是否显示:[yes|no]" << std::endl;
	cin>>status_show;
	if(strcmp(status_show, "yes") == 0) //两者相等则返回0，否则为其他值
	{
		SHOW_ON = true;//显示图片视频
	}
	else if(strcmp(status_show, "no") == 0)
		{
			SHOW_ON = false;//不显示图片视频
		}
		else
		{
			cout<<"\n输入信息有误\n";
			return -1;
		}
	
	//创建文件夹
	string file_address(argv[1], argv[1] + strlen(argv[1]));
	std::string SaveDepthAddress = file_address+"depth/";
	std::string SaveRGBAddress = file_address+"rgb/";
	if (argc == 2)
	{
		int return_num = createDirectory(argv[1]);//创建储存主文件夹
		createDirectory(SaveDepthAddress);//创建储存深度图文件夹
		createDirectory(SaveRGBAddress);//创建储存rgb图文件夹
	}
	else
	{
		cout<<"\n输入信息有误,不能创建文件夹\n";
		return -1;
	}

//搜寻并初始化传感器
   if(freenect2.enumerateDevices() == 0)
   {
       std::cout << "no device connected!" << std::endl;
       return -1;
   }
   string serial = freenect2.getDefaultDeviceSerialNumber();
   std::cout << "SERIAL: " << serial << std::endl;

//配置传输格式
#if 1 // sean
   int depthProcessor = Processor_cl;
   if(depthProcessor == Processor_cpu)
   {
       if(!pipeline)
           //! [pipeline]
           pipeline = new libfreenect2::CpuPacketPipeline();
       //! [pipeline]
   }
   else if (depthProcessor == Processor_gl) // if support gl
   {
#ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
       if(!pipeline)
       {
           pipeline = new libfreenect2::OpenGLPacketPipeline();
       }
#else
       std::cout << "OpenGL pipeline is not supported!" << std::endl;
#endif
   }
   else if (depthProcessor == Processor_cl) // if support cl
   {
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
       if(!pipeline)
           pipeline = new libfreenect2::OpenCLPacketPipeline();
#else
       std::cout << "OpenCL pipeline is not supported!" << std::endl;
#endif
   }



//启动设备
   if(pipeline)
   {
       dev = freenect2.openDevice(serial, pipeline);
   }
   else
   {
       dev = freenect2.openDevice(serial);
   }
   if(dev == 0)
   {
       std::cout << "failure opening device!" << std::endl;
       return -1;
   }
   signal(SIGINT, sigint_handler);
   protonect_shutdown = false;
   libfreenect2::SyncMultiFrameListener listener(
           libfreenect2::Frame::Color |
           libfreenect2::Frame::Depth |
           libfreenect2::Frame::Ir);
   libfreenect2::FrameMap frames;
   dev->setColorFrameListener(&listener);
   dev->setIrAndDepthFrameListener(&listener);


//启动数据传输
   dev->start();

   std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
   std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;




//循环接收
   libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
   libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4), depth2rgb(1920, 1080 + 2, 4);


   Mat rgbmat, depthmat, depthmatUndistorted, irmat, rgbd, rgbd2;
   cv::Mat depth1(480, 640, CV_8UC1);//深度图转换
   
       if( SHOW_ON == true )
	{
		cv::namedWindow("rgb", WND_PROP_ASPECT_RATIO);
		//cv::namedWindow("ir", WND_PROP_ASPECT_RATIO);
		cv::namedWindow("depth", WND_PROP_ASPECT_RATIO);
		//cv::namedWindow("undistorted", WND_PROP_ASPECT_RATIO);
		//cv::namedWindow("registered", WND_PROP_ASPECT_RATIO);
		//cv::namedWindow("depth2RGB", WND_PROP_ASPECT_RATIO);
	}
   
//与视频深度帧的帧率相对应
	std::ofstream f1;
	string TS_file_address = file_address +"depth .txt";
	f1.open(TS_file_address.c_str(), ios::out|ios::trunc);//覆盖，不存在则创建
	if(!f1.fail())
	{
		f1 << "# depth maps" << std::endl;
		f1 << "# file: 'Kinect2.0_###.zip'" << std::endl;
		f1 << "# timestamp filename" << std::endl;
	}
	f1 << std::fixed;
	
	//与视频的帧率相对应
	std::ofstream f;
	TS_file_address = file_address +"rgb.txt";
	f.open(TS_file_address.c_str(), ios::out|ios::trunc);//覆盖，不存在则创建
	if(!f.fail())
	{
		f << "# color images" << std::endl;
		f << "# file: 'Kinect2.0_###.zip'" << std::endl;
		f << "# timestamp filename" << std::endl;
	}
	f << std::fixed;
		
	int readyStream_RGB1 = -1;
	gettimeofday(&tv, NULL);
	Time0 = tv.tv_sec * 1000 + tv.tv_usec / 1000; // 毫秒

   while(!protonect_shutdown)
   {
       listener.waitForNewFrame(frames);
       libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
       //libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
       libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
       
        //保存RGB的时间戳
	cout << rgb->timestamp << endl;
	Time_frame = rgb->timestamp;
	Time_frame = Time_frame/10000.0;//转换为秒
	double current_Time = 2001021000.0;
	current_Time = current_Time+Time_frame;
	f << current_Time;
	f << " ";
	f << "rgb/" << std::to_string(current_Time) << ".png" << std::endl;
	std::stringstream  ImageName;
	ImageName << std::to_string(current_Time) << ".png";
	std::string SaveFilesName = SaveRGBAddress+ImageName.str();
	cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data).copyTo(rgbmat);
	int Image_size_Max = fmax(rgb->height,rgb->width);

	if(Image_size_Max > 640)
	{
		float scale = (float)640.0f/(float)Image_size_Max; 
		cv::resize(rgbmat,frameResized,cv::Size(),scale,scale);
	}
	else	
		frameResized = rgbmat;
	cv::imwrite(SaveFilesName,frameResized);
	
	Time_frame = depth->timestamp;
	Time_frame = Time_frame/10000.0;//转换为秒
	current_Time = 2001021000.0;
	current_Time = current_Time+Time_frame;
	f1 << current_Time;
	f1 << " ";
	f1 << "depth/" << std::to_string(current_Time) << ".png" << std::endl;
	std::stringstream  DepthName;
	DepthName << std::to_string(current_Time) << ".png";
	string file_address(argv[1], argv[1] + strlen(argv[1]));
	SaveFilesName = SaveDepthAddress+DepthName.str();
	cv::Mat(depth->height, depth->width, CV_32FC1, depth->data).copyTo(depthmat);
	minMaxLoc(depthmat, &minVal, &maxVal);
	depthmat.convertTo(depth1, CV_8UC1, 255.0/(maxVal-minVal), -255.0*minVal/(maxVal-minVal));
	cv::imwrite(SaveFilesName,depth1);
	
        //cv::Mat(ir->height, ir->width, CV_32FC1, ir->data).copyTo(irmat);

	
	
       // drawText(rgbmat, "NIT");
       if( SHOW_ON == true )
	{
		cv::imshow("rgb", rgbmat);
		//cv::imshow("ir", irmat / 4500.0f);
		cv::imshow("depth", depth1);
	}

       registration->apply(rgb, depth, &undistorted, &registered, true, &depth2rgb);

       //cv::Mat(undistorted.height, undistorted.width, CV_32FC1, undistorted.data).copyTo(depthmatUndistorted);
       //cv::Mat(registered.height, registered.width, CV_8UC4, registered.data).copyTo(rgbd);
       //cv::Mat(depth2rgb.height, depth2rgb.width, CV_32FC1, depth2rgb.data).copyTo(rgbd2);
	/*
       if( SHOW_ON == true )
	{
		cv::imshow("undistorted", depthmatUndistorted / 4500.0f);
		cv::imshow("registered", rgbd);
		cv::imshow("depth2RGB", rgbd2 / 4500.0f);
	}
       */
       int key = cv::waitKey(1);
       protonect_shutdown = protonect_shutdown || (key > 0 && ((key & 0xFF) == 27)); // shutdown on escape

       listener.release(frames);
   }
//关闭设备
   dev->stop();
   dev->close();

   delete registration;

#endif

   std::cout << "Goodbye World!" << std::endl;
   return 0;
}



/*
void drawText(Mat & image, const char* str)
{
   putText(image, str,
           Point(100, 100),
           FONT_HERSHEY_SIMPLEX, 3, // font face and scale
           Scalar(255, 255, 0), 
           3, LINE_AA); // line thickness and type
}
*/
