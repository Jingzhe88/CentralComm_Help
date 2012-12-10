#ifndef ROBOTVIDEORECEIVER_H_H
#define ROBOTVIDEORECEIVER_H_H 



#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/nonfree/features2d.hpp>

#include <iostream>

#include <Aria.h>
#include "ArNetworking.h"
using namespace std;
using namespace cv;


extern char* server_ip;
extern ArClientBase *client;
extern Mat robot_img;

void C_RobotVideoCB(ArNetPacket* robVideoPack);

class RobotVideo : public ArASyncTask
{
public:
	~RobotVideo()
	{
		robotVideo.release();
	};
  void* runThread(void*) ;
private:
	VideoWriter robotVideo;

};



#endif