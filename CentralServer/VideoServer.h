#ifndef VIDEOSERVER_H_H
#define VIDEOSERVER_H_H


#include "Aria.h"
#include "ArNetworking.h"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;

extern int CameraMoveCount;


void clientCloseCallback(ArServerClient * serverClient);

class VideoServerBase : public ArASyncTask
{


	

  void* runThread(void*) ;
public:
	static void RobotVideoCB (ArServerClient *, ArNetPacket*);
};

#endif