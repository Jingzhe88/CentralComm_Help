#ifndef VCCHANDLER_H
#define VCCHANDLER_H

#include "Aria.h"

extern ArVCC4* G_PTZHandler;

class VCCHandler
{
public:
	VCCHandler(ArRobot *robot);
private:
	ArVCC4 myPTU;
	ArRobot *myRobot;
};


#endif