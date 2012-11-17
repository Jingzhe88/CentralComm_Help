#ifndef GLASSESVIDEO_H_H
#define GLASSESVIDEO_H_H

#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "Aria.h"
using namespace cv;
using namespace std;

enum GLASSESMODE{IDLE=1, GLASSES_OR, ROBOT_SEARCH, GLASSES_CONTROL, TARGET_APPROACH, GLASSESCONFIRM };
enum RSGLASSESMODE{nod=1, shake, others};

// void *glassesVideo(void*); 
class GlassesVideo : public ArASyncTask
{
  void* runThread(void*) ;
};

#endif