#ifndef GLASSESVIDEO_H_H
#define GLASSESVIDEO_H_H

#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "ObjectRecognition.h"
#include "RobotVideoReceiver.h"
#include "Aria.h"
using namespace cv;
using namespace std;

enum GLASSESMODE{IDLE=1, GLASSES_OR, ROBOT_SEARCH, GLASSES_CONTROL, TARGET_APPROACH, GLASSESCONFIRM };
enum RSGLASSESMODE{nod=1, shake, others};

// void *glassesVideo(void*); 
class GlassesVideo : public ArASyncTask
{
public:
	GlassesVideo():gl_capture(3), gl_or("g20111105_4.yml.gz")
	{
	};
	~GlassesVideo()
	{
		gl_capture.release();
		glassesVideo.release();
		
	};
  void* runThread(void*) ;
private:
	VideoWriter glassesVideo;
	VideoCapture gl_capture; 
	ObjectRecognition gl_or;
};

#endif