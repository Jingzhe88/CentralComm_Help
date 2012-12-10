#ifdef WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif

#include "CommReceiver.h"
#include "RobotVideoReceiver.h"
#include "glassesVideo.h"
#include "GlassesMotion\CircularBuffer.h"
#define NO_EXPMT 0

// Including SDKDDKVer.h defines the highest available Windows platform.

// If you wish to build your application for a previous Windows platform, include WinSDKVer.h and
// set the _WIN32_WINNT macro to the platform you wish to support before including SDKDDKVer.h.

#include <SDKDDKVer.h>


#include <sapi.h>
#include <stdio.h>
#include <tchar.h>
#include <iostream>


#define _ATL_APARTMENT_THREADED

#include <atlbase.h>
extern CircularBuffer CB;
extern CComModule _Module;
#include <atlcom.h>

bool turnActionIsFinished = false;
extern char* server_ip;
extern ArMutex mutex_robotVideo;
extern Mat robot_img;
extern int G_Target;
extern ArClientBase *client;

int G_Search_Step;
bool isDoneRobot =false;
extern GLASSESMODE G_glassesMode;
extern ArMutex GlassesModeMutex;
//time_t HelpStartTime, HelpEndTime, elapseTime;
ArTime expmtTimer;
int robotCameraAngle = 0;

/**
* ATTENTION: The prototype for speech, users should not use it anymore.

* Use this function to speak Object's name.
*
* @param index is the number of object that between 0 and 119 for Oct.14 version.
* @return true if worked perfectly, otherwise false.
*/
int robotSpeakObjName(int index){
	
	if(index > 119 || index < 0){
		std::cout << "robotSpeekObjName: The index exceed normal value." << std::endl;
		exit(1);
		return false;
	}
	int choice = index / 10;
	ISpVoice * pVoice = NULL;
	LPCWSTR objectName[12] = {L"Wireless Router", L"Small Car", L"Coffee Cup", L"Paper Box", L"Tea Container",
							  L"Mr Potato Head", L"Toy Train", L"Big Bottle", L"Pencil Case", L"Small Bottle",
							  L"Tooth Paste", L"Tooth Paste"};


    if (FAILED(::CoInitialize(NULL)))
        return false;

    HRESULT hr = CoCreateInstance(CLSID_SpVoice, NULL, CLSCTX_ALL, IID_ISpVoice, (void **)&pVoice );
    if( SUCCEEDED( hr ) )
    {
        hr = pVoice->Speak(objectName[choice], 0, NULL);
        pVoice->Release();
        pVoice = NULL;
    }
	
    ::CoUninitialize();
    return true;
}

/**
* Use this function to speak Object's name.
*
* @param: index is the number of object that between 0 and 119 for Oct.14 version.
* @return: if worked perfectly the function will return 1, otherwise 0.
*/

int robotSpeak(const int index, const char *option)
{

	ISpVoice * pVoice = NULL;
	LPCWSTR objectName[4] = {L"Truck", L"Motor", L"Toy ", L"flower pot "};


    if (FAILED(::CoInitialize(NULL)))
        return false;

    HRESULT hr = CoCreateInstance(CLSID_SpVoice, NULL, CLSCTX_ALL, IID_ISpVoice, (void **)&pVoice );
    if( SUCCEEDED( hr ) )
    {
			if(!strcmp(option, "name"))
			{
				if(index > 19 || index < 0)
				{
				std::cout << "robotSpeekObjName: The index exceed normal value." << std::endl;
				exit(1);
				return false;
				}
				int choice = index / 5;
        hr = pVoice->Speak(objectName[choice], 0, NULL);
			}

			if(!strcmp(option, "help"))
				hr = pVoice->Speak(L"I Need Help", 0, NULL);
			if(!strcmp(option, "idle"))
				hr = pVoice->Speak(L"back to idle", 0, NULL);
			if(!strcmp(option,  "cancel"))
				hr = pVoice->Speak(L"canceled   go back to idle", 0, NULL);
			if(!strcmp(option,  "OR_Entry"))
				hr = pVoice->Speak(L"Object Recognition mode", 0, NULL);
			if(!strcmp(option,  "complete"))
				hr = pVoice->Speak(L"Mission has been completed", 0, NULL);
			if(!strcmp(option,  "tagetApproach"))
				hr = pVoice->Speak(L"heading to target", 0, NULL);
      pVoice->Release();
      pVoice = NULL;
    }

    ::CoUninitialize();

    return true;
}

void asking_for_help()
{
	
	robotSpeak(0,"help");
	
	cout << "the robot needs help!" << endl;
	CB.clear();
	G_glassesMode = GLASSES_CONTROL;
  ISpVoice * pVoice = NULL;
}


void RobotCommand(int robotMoveComm, int cameraAngle )
{
	ArNetPacket pack;
	/*robotMoveComm = 255;*/
	if (client->getRunningWithLock())
	{
		switch(robotMoveComm)
		{
		case 0:
			//client.requestOnce("search_mode");
			break;
		case 1: 
			client->requestOnce("RobotMotion");
			cout << "RobotMotion has been sent out!" << endl;
			break;
		case 2:
			client->requestOnce("CameraMotion");
			cout << "CameraMotion has been sent out!" << endl;
			break;
		case 3:
			if(!turnActionIsFinished)
			{
			client->requestOnce("RobotTurnLeft");
			turnActionIsFinished = true;
			//elapseTime=0;
			cout << "RobotTurnLeft has been sent out!" << endl;
			}
			break;
		case 4:
			if(!turnActionIsFinished)
			{
			client->requestOnce("RobotTurnRight");
			turnActionIsFinished = true;
			//elapseTime=0;
			cout << "RobotTurnRight has been sent out!" << endl;
			}
			break;
		case 5: //CTargetApproach
			robotSpeak(0,"tagetApproach");
			client->requestOnce("TargetApproach");
			//client->requestOnce("TargetApproach");
			cout << "TargetApproach has been sent out!" << endl;
			break;

		case 6: 
			pack.doubleToBuf(robotCameraAngle);
			client->requestOnce("Calibration", &pack);
			cout << "Calibration is sent out!" << endl; 
			break;

		case 7:
			robotSpeak(255, "cancel");
			client->requestOnce("GlassesCancel");
			isDoneRobot = true;
			G_glassesMode = IDLE;
			cout << "Cancel has been sent out!" << endl;
			break;
		case 8: //CTargetApproachObstacles
			robotSpeak(0,"tagetApproach");
			client->requestOnce("TargetApproachObstacles");
			cout << "TargetApproachObstacles has been sent out!" << endl;
			break;
		case 9: 
			pack.doubleToBuf(robotCameraAngle);
			client->requestOnce("Calibration", &pack);
			cout << "Calibration is sent out!" << endl; 
			break;
		case 255: //for testing
			cout << "You are calling the Legacy function!" <<endl;
			client->requestOnce("turn");
			cout << "TURN is sent out!" << endl; 
			break;
		}	
	}
}


/************************************************************************/
/*                    Callback Function Group							              */
/************************************************************************/      
void C_Calibration(ArNetPacket * pack)
{
	cout << "Calibration is finished!" << endl; 
						
	RobotCommand(CTargetApproach);
}

void C_RobotTurnLeft(ArNetPacket * pack)
{
	turnActionIsFinished = false;
	cout << "Turn left is finished!" << endl; 
}

void C_RobotTurnRight(ArNetPacket * pack)
{
	turnActionIsFinished = false;
	cout << "Turn right is finished!" << endl; 
}

void C_RobotMotion(ArNetPacket * pack)
{
	
	cout << "RobotMotion has been finished!" << endl;
	G_Search_Step++;
	isDoneRobot = true;
	
	//RobotCommand(2); //CameraMotion
}

void C_CameraMotion(ArNetPacket * pack)
{
	cout << "Camera motion is back!" <<endl;
	G_Search_Step++;
	isDoneRobot = true;
}

void C_TargetApproach(ArNetPacket * pack)  //finish target approach
{
	cout << "Total Time for finding the object: " << expmtTimer.mSecSinceLL()<<endl;
	cout << "Final step completed: Arrive the object!!!" << endl; 
	robotSpeak(0,"complete");
	GlassesModeMutex.lock();
	G_glassesMode = IDLE;
	robotSpeak(0,"idle");
	G_Search_Step = 0;
	isDoneRobot = true;
	GlassesModeMutex.unlock();
	
}


void C_TargetApproach_Obstacles(ArNetPacket * pack)  //finish target approach avoid middle obstacle
{
	cout<< "C_TargetApproach_Obstacles is back" <<endl;
	GlassesModeMutex.lock();
	G_glassesMode = ROBOT_SEARCH;
	GlassesModeMutex.unlock();
	G_Search_Step = 0;
	isDoneRobot = true;
}
//---------------------------------------------------------------------------------------------




RobotSearch::RobotSearch():robotOR("r20121111_4.yml.gz")
{

}

void RobotSearch::moveRobotCamera()
{
#if NO_EXPMT
	if(G_Search_Step<=13)
	{
		//the object is detected, enter the search mode. callback: S_RobotMotion
		RobotCommand(CameraMotion); //cameraMotion
		isDoneRobot = false;
	}
	if(G_Search_Step>13)
	{

		asking_for_help();
		G_Search_Step = 0;
		//isDoneRobot = false;
	}
#else
	expmtTimer.setToNow();
	asking_for_help();
#endif
}

void RobotSearch::resetRobotCameraParam()
{
	G_Search_Step=0;
	isDoneRobot = true;
}

bool RobotSearch::OR()
{
	int robot_object[3]={255,255,255};
	Mat robot_img_backup;
	for(int i=0;i<3;i++)
	{
		mutex_robotVideo.lock();
		robot_img.copyTo(robot_img_backup);
		mutex_robotVideo.unlock();
		robot_object[i]=255;
		cout << "-- robot OR -- " <<endl;
		robotCameraAngle=0;
		robot_object[i] = robotOR.find(robot_img_backup, 'R', robotCameraAngle);
		if (robot_object[i]!=255)
		{
			robot_object[i] /= 5;
		}
	}
	if ((robot_object[0] == G_Target || robot_object[1] == G_Target || robot_object[2] == G_Target) 
		&& G_Target!=255 /*&& G_glassesMode == ROBOT_SEARCH*/) //object is detected
		return true;
	else 
		return false;
}


void* RobotSearch::runThread(void*)
{
	while(1) 
  {
		if(G_glassesMode == ROBOT_SEARCH)
		{
			if(OR())
			{
				RobotCommand(Calibrate);
				G_glassesMode = TARGET_APPROACH;
			}
			else if(isDoneRobot)
			{
				moveRobotCamera();
			}


			//HelpEndTime = time(NULL);
			//elapseTime = HelpEndTime - HelpStartTime;
		}

		//GlassesModeMutex.lock();
		//if(elapseTime >= 45 /*1min*/ && ( G_glassesMode==ROBOT_SEARCH/*||G_glassesMode==GLASSES_CONTROL*/) )
		//{
		//		asking_for_help();
		//}
		//GlassesModeMutex.unlock();
		
  }//end of while(1)
}




//--------------Legacy---------------
//void CommReceiver(ArNetPacket * pack)
//{
//    ObjectRecognition robot_or;
////     cout << "Next command is computing." << endl;
//    //mutex_robotVideo.lock();
//    //Mat robot_img_copy = robot_img.clone();
//    //mutex_robotVideo.unlock();
//    
//    //object detection from robot view.
//    int robotObjResult = 255;
//    Sleep(3);		//3s
//    //robotObjResult = robot_or.find(robot_img_copy);
//    
//    
//	
//    if (255 != robotObjResult )
//    {
//      //object has been detected, go forward!
//      RobotCommand(2);
//    }
//    else
//    {
//      RobotCommand(1);
//    }
//
//  
//}


//--------------------------Legacy-------------------------------------
//       for the database of 120 small objects 
//---------------------------------------------------------------------

//int robotSpeakObjName(int index){
//	
//	if(index > 119 || index < 0){
//		std::cout << "robotSpeekObjName: The index exceed normal value." << std::endl;
//		exit(1);
//		return false;
//	}
//	int choice = index / 10;
//	ISpVoice * pVoice = NULL;
//	LPCWSTR objectName[12] = {L"Wireless Router", L"Small Car", L"Coffee Cup", L"Paper Box", L"Tea Container",
//							  L"Mr Potato Head", L"Toy Train", L"Big Bottle", L"Pencil Case", L"Small Bottle",
//							  L"Tooth Paste", L"Tooth Paste"};
//
//
//    if (FAILED(::CoInitialize(NULL)))
//        return false;
//
//    HRESULT hr = CoCreateInstance(CLSID_SpVoice, NULL, CLSCTX_ALL, IID_ISpVoice, (void **)&pVoice );
//    if( SUCCEEDED( hr ) )
//    {
//        hr = pVoice->Speak(objectName[choice], 0, NULL);
//        pVoice->Release();
//        pVoice = NULL;
//    }
//	
//    ::CoUninitialize();
//    return true;
//}