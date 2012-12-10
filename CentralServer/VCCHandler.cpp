#include "VCCHandler.h"

ArVCC4* G_PTZHandler;
VCCHandler::VCCHandler(ArRobot *robot) :
	myPTU(robot,false,ArVCC4::COMM_BIDIRECTIONAL,true,true,ArVCC4::CAMERA_C50I)
{
	G_PTZHandler = &myPTU;
	myPTU.setLEDControlMode(2);
	myPTU.enableIRFilterMode();
	myPTU.enableIRLEDs();
	myPTU.enableAutoUpdate();
}