#ifdef WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif

#include "glassesVideo.h"
#include "ObjectRecognition.h"
#include "RobotVideoReceiver.h"
#include "CommReceiver.h"
#include "GlassesMotion.h"

#include "CommReceiver.h"

extern CircularBuffer CB;
// int GlassesMode;												//from clientMain.cpp			0:Idle, 1: OR, 2:Control
extern ArMutex GlassesModeMutex;							//from clientMain.cpp
extern GLASSESMODE G_glassesMode;//1 idle, 2 OR, 3 control, 4 Cancel
extern int G_Search_Step;
extern bool isDoneRobot;
//extern ArTime helpTimer;

extern time_t HelpStartTime;
ArMutex GlassesVideoMutex;

int G_Target = 255; //from glassesVideo.cpp
RobotSearch *G_robotsearch;



//class glassesOR : public ArASyncTask
//{
//	ObjectRecognition gl_or;
//	Mat gl_img_bk, *gl_img;
//	Mat glres_image; //display result image
//	stringstream ret_src1; //var for src of result
//	int gl_result;
//public:
//	glassesOR(Mat *_gl_img)
//	{
//		//gl_img = _gl_img;
//	}
//	void* runThread(void*) 
//	{
//		//-------------------------object recognition ------------------------
//
//
//		return 0;
//	}
//
//	//void stopRunning(void)
//	//{
//	//	destroyWindow("Grey Image");
//	//}
//
//};


void* GlassesVideo::runThread(void*) 
{

	VideoCapture gl_capture(3); 

	gl_capture.set(CV_CAP_PROP_FRAME_WIDTH , 640);
	gl_capture.set(CV_CAP_PROP_FRAME_HEIGHT, 480);

	if(!gl_capture.isOpened())
	{
		cout << "Cannot open glasses video !" << endl;
	}

	Mat gl_img, gl_img_OR;

	Mat curMat, preMat;

	//glassesOR glOR(&gl_img_OR);
	//glOR.stopRunning();
	ObjectRecognition gl_or("g20111105_4.yml.gz");
	Mat gl_img_bk;
	Mat glres_image;				//display result image

	int gl_result=255;



	RobotSearch robotsearch;
	G_robotsearch = &robotsearch;
	//robotsearch.create();
	robotsearch.stopRunning();

	
	//namedWindow("Glasses Video");
	//moveWindow("Glasses Video", 645, 0);
	namedWindow("Video Live");
	moveWindow("Video Live", 645, 0);
	namedWindow("Glasses_result",CV_WINDOW_NORMAL);
	moveWindow("Glasses_result",1000,600);
	//G_glassesMode = GLASSES_OR;
	while(1)
	{

		gl_capture >> gl_img;
		cvtColor(gl_img,gl_img_bk,CV_RGB2GRAY);
		imshow("Video Live",gl_img_bk);
		waitKey(1);

		//----------------------------glasses Motion ------------------------
		preMat = gl_img.clone();
		//imshow("preMat", preMat);
		gl_capture >> curMat;
		//imshow ("cur", curMat);
		modeSwitch(preMat, curMat);
		//-------------------------------------------------------------------

		if(G_glassesMode == GLASSES_OR) //OR MODE
		{

			gl_result=255;
			gl_result = gl_or.find(gl_img_bk, 'G');

			//if(gl_result !=255)
			//{
			//	gl_capture >> gl_img;
			//	cvtColor(gl_img,gl_img_bk,CV_RGB2GRAY);
			//	imshow("Video Live",gl_img);
			//	waitKey(1);
			//	gl_result=255;
			//	gl_result = gl_or.find(gl_img_bk, 'G');
			//	/*if(gl_result !=255)
			//	{
			//		gl_capture >> gl_img;
			//		cvtColor(gl_img,gl_img_bk,CV_RGB2GRAY);
			//		imshow("Video Live",gl_img);
			//		waitKey(1);
			//		gl_result=255;
			//		gl_result = gl_or.find(gl_img_bk, 'G');
			//	}
			//	else gl_result=255;*/

			//}

			if(gl_result !=255)
			{

				//-------------------------Display the result ------------------------
				robotSpeak(gl_result, "name");
				stringstream ret_src1;  //result src
				ObjectRecognition::loadImage(ret_src1, gl_result, 'G', 1);
				glres_image = imread(ret_src1.str());
				imshow("Glasses_result", glres_image);
				waitKey(1);
				
				//--------------------glasses goes to robot search mode------------------
				GlassesModeMutex.lock();
				CB.clear();
				G_glassesMode = ROBOT_SEARCH;
				G_Search_Step = 0;
				isDoneRobot = true;
				G_Target= gl_result/5;
				gl_result = 255;
				//HelpStartTime = time(NULL);
				GlassesModeMutex.unlock();

				////-------------------------Open robot search thread ------------------------

				if(!robotsearch.getRunning())
					robotsearch.runAsync();
				
			}

		}



	}
	//return 0;

}