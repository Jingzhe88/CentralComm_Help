

#include "MotionsServer.h"
#define ANGAL_FOR_TURN 13

const double PI = 3.1415926;
double targetDistance =0.0;
extern ArPathPlanningTask* G_PathPlanning;
extern ArRobot robot;
extern ArSick sick;


extern ArVCC4* G_PTZHandler;		//from KeyPtz.cpp
// extern ArAnalogGyro *G_gyro;

int goals[8] = {-800,2200,  -200,2000,  600,600,  0,-450};
int headings[4] = {0,90,180,0};

int findNeighbor(int *goals);
int goalIndex = 0;
int headingIndex = 0;
int CameraMoveCount = 0;
void CoordinateCalculation(const double robotCurrentX, const double robotCurrentY, double* targetX, double* targetY, const double camAngle, const double robotHeading, const double movDistance);
void coordinateCalculation(double robotCurrentX,double robotCurrentY,double *outputX,double *outputY,double camAngle, double robotHeading,double moveDistance);
void basic_turn(int turnAngal)
{
	// ArTime start;
	// G_PTZHandler->reset();
	//ArUtil::sleep(500);
	//G_PTZHandler->tiltRel(-10);
	//CameraMoveCount=0;
	//robot.lock();

	double robotHeading = robot.getPose().getTh();
	robotHeading += turnAngal;
	double robotCurrentX = robot.getPose().getX() ;
	double robotCurrentY = robot.getPose().getY();

	G_PathPlanning->pathPlanToPose(ArPose(robotCurrentX, robotCurrentY , robotHeading),true,true);
	while(G_PathPlanning->getState() != ArPathPlanningTask::REACHED_GOAL );


	//robot.setHeading(robot.getTh()+turnAngal);
	// robot.unlock();

	//start.setToNow();
	//while (1)
	//{
	// robot.lock();
	// if (robot.isHeadingDone())
	// {
	//  printf(" Finished turn\n");
	//  
	//  ArUtil::sleep(50);
	//  cout << " current heading: " << "    " << robot.getTh()<<endl;
	//	robot.unlock();
	//  break;
	// }
	// if (start.mSecSince() > 5000)
	// {
	//  printf(" Turn timed out\n");
	//  
	//  cout << " current heading: " << "    " << robot.getTh()<<endl;
	//	robot.unlock();
	//  break;
	// }
	// robot.unlock();
	// ArUtil::sleep(10);
	//}  
}

void S_GlassesCancel( ArServerClient *serverclient, ArNetPacket *socket)
{
	cout <<"Canceled from glasses, set back camera" <<endl;
	goalIndex = 0;
	headingIndex = 0;
	CameraMoveCount = 0;
	targetDistance = 0;
	G_PTZHandler->reset();
	ArUtil::sleep(500);
	//G_PTZHandler->zoom(G_PTZHandler->getZoom() + 250);
	G_PTZHandler->tiltRel(-10);
	serverclient->sendPacketTcp(socket);
}

void S_Calibration( ArServerClient *serverclient, ArNetPacket *socket)
{
	//G_PTZHandler->zoom(G_PTZHandler->getZoom() + 1300);
	//ArUtil::sleep(8000);
	int cameraAdjustingAngle = socket->bufToDouble();
	cout << "Adjust camera angle ... " << " " <<
		cameraAdjustingAngle << endl;

	G_PTZHandler-> panRel(cameraAdjustingAngle);
	ArUtil::sleep(4000);
	serverclient->sendPacketTcp(socket);
}





void S_CameraMotion( ArServerClient *serverclient, ArNetPacket *socket)
{

	G_PTZHandler->getRealPanTilt();
	if(G_PTZHandler->getTilt()>=0)
		G_PTZHandler->tiltRel(-10);

	if(CameraMoveCount < 6)
	{	
		G_PTZHandler->panRel(-10);
	}
	else
	{
		if (CameraMoveCount==6) 
			G_PTZHandler->panRel(60);
		else 
			G_PTZHandler->panRel(10);
	}
	CameraMoveCount++;
	if (CameraMoveCount>13)
	{
		CameraMoveCount=0;G_PTZHandler->panRel(-70);
	}
	//------------------------------------------------------------------------------------------------------
	ArUtil::sleep(2500);
	//cout << timer.mSecSince() <<endl;
	G_PTZHandler->getRealPanTilt();
	cout << " Current pan = " << G_PTZHandler->getPan() << endl;
	serverclient->sendPacketTcp(socket);

}



void S_RobotTurnLeft( ArServerClient *serverclient, ArNetPacket *socket)
{
	cout << "turn left" <<endl;
	G_PTZHandler->reset();
	ArUtil::sleep(300);
	G_PTZHandler->tiltRel(-10);
	basic_turn(ANGAL_FOR_TURN);
	serverclient->sendPacketTcp(socket);
}


void S_RobotTurnRight( ArServerClient *serverclient, ArNetPacket *socket)
{
	cout << "turn right" <<endl;
	G_PTZHandler->reset();
	ArUtil::sleep(300);
	G_PTZHandler->tiltRel(-10);
	basic_turn(-ANGAL_FOR_TURN);
	serverclient->sendPacketTcp(socket);
}


void S_RobotMotion( ArServerClient *serverclient, ArNetPacket *socket)
{
	cout << "--------------Path Planning Information--------------------" <<endl;
	cout << "SafeCollisionRange = " << G_PathPlanning->getSafeCollisionRange()  << endl;
	cout << "FrontClearance     = " << G_PathPlanning->getFrontClearance()      << endl;
	G_PathPlanning->setCollisionRange(500);
	G_PathPlanning->setFrontClearance(40);
	//-----------------------Formal Code for RobotMotion-------------------------------------------------  
	G_PTZHandler->reset();
	ArUtil::sleep(300);
	G_PTZHandler->tiltRel(-10);
	CameraMoveCount=0;
	if(goalIndex <= 7)
	{
		cout << robot.getPose().getX() << endl;
		cout << robot.getPose().getY() << endl;


		G_PathPlanning->pathPlanToPose(ArPose(goals[goalIndex++],goals[goalIndex++],headings[headingIndex++]), true,true);
		cout << "RobotMotion is processing..." <<endl;

		while(G_PathPlanning->getState() != ArPathPlanningTask::REACHED_GOAL )
		{
			if (G_PathPlanning->getState() == ArPathPlanningTask::ABORTED_PATHPLAN)
			{ G_PathPlanning->cancelPathPlan();break;}
			else if(G_PathPlanning->getState() == ArPathPlanningTask::FAILED_PLAN)
			{G_PathPlanning->pathPlanToPose(ArPose(-300,-100,0),true,true);}
		}

	}
	else
	{
		goalIndex =0;
		headingIndex = 0;

	}   
	serverclient->sendPacketTcp(socket);
	//-------------------------------------------------------------------------------------------------------

}

void S_TargetApproach( ArServerClient *serverclient, ArNetPacket *socket)
{
	cout << "The last step: TargetApproach!" <<endl;

	//G_PathPlanning->setCollisionRange(1000);
	//G_PathPlanning->setFrontClearance(40);
	//G_PathPlanning->setGoalDistanceTolerance(2500);
	G_PTZHandler->getRealPanTilt();
	ArUtil::sleep(300);
	double camAngle = -1 * G_PTZHandler->getPan();
	double robotHeading = robot.getPose().getTh();
	double robotCurrentX = robot.getPose().getX() ;
	double robotCurrentY = robot.getPose().getY();
	double angle = 0.0;
	double distance =0.0;

	double targetX, targetY;
	int disThreshold = 500;
	//test mode
	//     G_PTZHandler->panRel(-30);
	//G_PathPlanning->setFrontClearance(40);
	//G_PathPlanning->setObsThreshold(1);

	cout << "--------------Path Planning Information--------------------" <<endl;
	cout << "SafeCollisionRange = " << G_PathPlanning->getSafeCollisionRange()  << endl;
	cout << "FrontClearance     = " << G_PathPlanning->getFrontClearance()      << endl
		<< "GoalDistanceTolerance = " << G_PathPlanning->getGoalDistanceTolerance() << endl
		<< "setGoalOccupiedFailDistance = " << G_PathPlanning->getGoalOccupiedFailDistance() << endl
		<< "getObsThreshold = " << G_PathPlanning->getObsThreshold() <<endl
		<< "getLocalPathFailDistance = " << G_PathPlanning->getLocalPathFailDistance() << endl;
	//getCurrentGoal

	//--------------- Set up the laser range device to read the distance from the target -----------------------------
	// for(int i =0; i< 20;i++)
	//sick.currentReadingPolar(robotHeading+ camAngle -2.5, robotHeading+ camAngle +2.5, &angle);

	sick.lockDevice();

	//if (distance == 0 || distance>disThreshold)
	distance = sick.currentReadingPolar(/*robotHeading+*/ camAngle -2.5, /*robotHeading+*/ camAngle +2.5, &angle)-disThreshold;
	//double distance = sick.currentReadingPolar(89, 90, &angle);
	cout << "The closest reading is " << distance << " at " << angle << " degree , " << "disThreshold : " <<disThreshold << " " << robotHeading << " " << camAngle<< endl;

	sick.unlockDevice();



	//----------------------------------------------------------------------------------------------------------------

	//basic_turn(camAngle);

	cout << "Camera Angle is " << camAngle << endl;
RECALCULATE:
	cout << "before calculation, check originalX, originalY " << robotCurrentX << " " << robotCurrentY << " robotHeading is " << robotHeading << endl<<endl;  
	coordinateCalculation(robotCurrentX,robotCurrentY,&targetX,&targetY,camAngle,robotHeading,distance);
	//
	//cout << "before movement, check targetX, target Y" << targetX << " " << targetY << "robotHeading is "<< robotHeading << endl << endl;


	cout << "targetX : " <<targetX << " targetY" <<targetY;
	G_PathPlanning->pathPlanToPose(ArPose(targetX,targetY,camAngle),true,true);
	cout << "RobotMotion is processing..." <<endl;

	G_PTZHandler->reset();
	ArUtil::sleep(300);
	G_PTZHandler->tiltRel(-10);
		while(1)
		{
			if (G_PathPlanning->getState() == ArPathPlanningTask::MOVING_TO_GOAL || G_PathPlanning->getState() == 1 || G_PathPlanning->getState() == 2)
				continue;
			else
			if(G_PathPlanning->getState() == ArPathPlanningTask::REACHED_GOAL )
			{
				break;
			}else //Failed Handler
			{
				/*if (G_PathPlanning->getState() == ArPathPlanningTask::FAILED_MOVE)
				{
					cout << "FAILED_MOVE"<<endl;
					G_PathPlanning->cancelPathPlan(); 
					break;
				}*/
				if(G_PathPlanning->getState() == ArPathPlanningTask::FAILED_PLAN)
				{
					cout << "FAILED_PLAN"<<endl;
					if (distance>200)
					{
						distance -= 100;
						goto RECALCULATE;
					}
					else break;
				}
				/*
				else if(G_PathPlanning->getLocalPathState() == ArPathPlanningTask::OBSTACLE_TOO_CLOSE )
				{
					cout << "OBSTACLE_TOO_CLOSE"<<endl;
					G_PathPlanning->cancelPathPlan();
					break;
				}

				else*/ if(G_PathPlanning->getLocalPathState() == ArPathPlanningTask::NO_LOCAL_PLAN  )
				{
					cout << "NO_LOCAL_PLAN"<<endl;
					if (distance>200)
					{
						distance -= 100;
						goto RECALCULATE;
					}
					else break;
				}
				else 
				{
					cout << "G_PathPlanning->getState() is " << G_PathPlanning->getState()<<endl;
					cout << "I have no idea what failure is... "<<endl;
					G_PathPlanning->cancelPathPlan();
					break;
				}
			}
		}
	//serverclient->sendPacketTcp(socket);

	ArUtil::sleep(2000);


	cout << "RobotMotion is heading home..." <<endl;
	G_PathPlanning->pathPlanToPose(ArPose(0,0,0),true,true);

		while(1)
		{
			if (G_PathPlanning->getState() == ArPathPlanningTask::MOVING_TO_GOAL || G_PathPlanning->getState() == 1 || G_PathPlanning->getState() == 2)
				continue;
			else
			if(G_PathPlanning->getState() == ArPathPlanningTask::REACHED_GOAL )
			{
				break;
			}else //Failed Handler
			{
				/*if (G_PathPlanning->getState() == ArPathPlanningTask::FAILED_MOVE)
				{
					cout << "FAILED_MOVE"<<endl;
					G_PathPlanning->cancelPathPlan(); 
					break;
				}*/
				if(G_PathPlanning->getState() == ArPathPlanningTask::FAILED_PLAN)
				{
					cout << "FAILED_PLAN"<<endl;

					 break;
				}
				/*
				else if(G_PathPlanning->getLocalPathState() == ArPathPlanningTask::OBSTACLE_TOO_CLOSE )
				{
					cout << "OBSTACLE_TOO_CLOSE"<<endl;
					G_PathPlanning->cancelPathPlan();
					break;
				}

				else*/ if(G_PathPlanning->getLocalPathState() == ArPathPlanningTask::NO_LOCAL_PLAN  )
				{
					cout << "NO_LOCAL_PLAN"<<endl;

					break;
				}
				else 
				{
					cout << "G_PathPlanning->getState() is " << G_PathPlanning->getState()<<endl;
					cout << "I have no idea what failure is... "<<endl;
					G_PathPlanning->cancelPathPlan();
					break;
				}
			}
		}
	serverclient->sendPacketTcp(socket);
}



double getAngle(double targetX, double targetY, double originalX, double originalY){
	if(targetX = originalX){
		if(targetY > originalY)
			return 90;
		else
			return -90;
	}

	if(targetY == originalY){
		if(targetX > originalX)
			return 0;
		else
			return -180;
	}

	if(targetX > originalX){
		return atan((targetY - originalY)/(targetX - originalX));
	}else{
		if(targetY > originalY)
			return -1*atan((targetY - originalY)/(targetX - originalX))+90;
		else
			return -1*atan((targetY - originalY)/(targetX - originalX))-90;
	}
}


#define DISTANCE_VALVE 1000
#define EXTRA_DISTANCE 500
ArPose OriginalPose;
void S_TargetApproach_Obstacles( ArServerClient *serverclient, ArNetPacket *socket)
{
	double camAngle = -1 * G_PTZHandler->getPan(); //negative #: right    positive #: left
	double angle = 0.0;
	double targetX, targetY;
	ArPose MidPose;

	if(targetDistance==0)
	{
		/*	firstTime = false;*/
		sick.lockDevice();
		targetDistance = sick.currentReadingPolar(camAngle - 2.5, camAngle + 2.5, &angle);

		sick.unlockDevice();
		OriginalPose = robot.getPose();
	}
	cout << "The closest reading is " << targetDistance << " at " << angle << " degree , " << " " << camAngle<< endl;
	coordinateCalculation(OriginalPose.getX(),OriginalPose.getX(),&targetX,&targetY,0, OriginalPose.getTh(),targetDistance);
	cout << "targetPose = " << targetX <<" " << targetY <<endl;

	targetDistance += EXTRA_DISTANCE;

	coordinateCalculation(OriginalPose.getX(),OriginalPose.getX(),&targetX,&targetY,0, OriginalPose.getTh(),targetDistance);

	cout << "targetPose = " << targetX <<" " << targetY <<endl;

	ArPose targetPose(targetX,targetY,OriginalPose.getTh());

	G_PathPlanning->pathPlanToPose(targetPose,true,true);	
	double tempDistance;
	ArUtil::sleep(1000);
	while(G_PathPlanning->getState() != ArPathPlanningTask::REACHED_GOAL )
	{
		//std::list<ArPose> path = G_PathPlanning->getPathFromTo(robot.getPose(), targetPose);

		std::list<ArPose> path = G_PathPlanning->getCurrentPath (robot.getPose());
		double tempX;
		double tempY;
		double tempTh;
		for(std::list<ArPose>::iterator myIterator = path.begin(); myIterator != path.end(); myIterator++)
		{
			tempX = myIterator->getX();
			tempY = myIterator->getY();
			tempTh = myIterator->getTh();
			cout << tempX  << " " << tempY << " " << tempTh << endl;

			tempDistance = sqrt((tempX - targetX)*(tempX - targetX) + (tempY - targetY)*(tempY - targetY));

			if(tempDistance <= DISTANCE_VALVE)
			{
				MidPose.setX(tempX);
				MidPose.setY(tempY);
				MidPose.setTh(myIterator->getTh());

				//double midAngle = getAngle(tempX, tempY, OriginalPose.getX(), OriginalPose.getY());
				//if(midAngle > OriginalPose.getTh())
				//	MidPose.setTh(OriginalPose.getTh() + 90);
				//else
				//	MidPose.setTh(OriginalPose.getTh() - 90);
				G_PathPlanning->cancelPathPlan();
				goto breakPathPlanning;
			}
		}
	}
breakPathPlanning:
	G_PathPlanning->pathPlanToPose(MidPose,true,true);
	cout << "TO mid pose"<<endl;
	while(G_PathPlanning->getState() != ArPathPlanningTask::REACHED_GOAL )
	{
		if (G_PathPlanning->getState() == ArPathPlanningTask::FAILED_MOVE)
		{
			G_PathPlanning->cancelPathPlan();cout <<  "x " << robot.getPose().getX()<< " y " <<robot.getPose().getY() <<endl; break;
		}
		else if(G_PathPlanning->getState() == ArPathPlanningTask::FAILED_PLAN)
		{

		}
	}
	serverclient->sendPacketTcp(socket);
}



void coordinateCalculation(double robotCurrentX,double robotCurrentY,double *outputX,double *outputY,double camAngle, double robotHeading,double moveDistance){
	double currentAngle;

	currentAngle = robotHeading+camAngle;
	
	if(currentAngle > 180)
	{
		currentAngle -= 360;
	}
	else if(currentAngle < -180)
	{
		currentAngle += 360;
	}

	if(currentAngle < 90 && currentAngle >= 0){
		*outputX = robotCurrentX + moveDistance * cos(currentAngle*3.1415926/180);
		*outputY = robotCurrentY + moveDistance * sin(currentAngle*3.1415926/180);
		return;
	}else if(currentAngle >= 90 && currentAngle <= 180){
		currentAngle = 180 - currentAngle;
		*outputX = robotCurrentX - moveDistance * cos(currentAngle*3.1415926/180);
		*outputY = robotCurrentY + moveDistance * sin(currentAngle*3.1415926/180);
		return;
	}else if(currentAngle >= -180 && currentAngle < -90){
		currentAngle = currentAngle - 180;
		*outputX = robotCurrentX - moveDistance * cos(currentAngle*3.1415926/180);
		*outputY = robotCurrentY - moveDistance * sin(currentAngle*3.1415926/180);
		return;
	}else if(currentAngle >= -90 && currentAngle < 0){
		currentAngle = 360 - currentAngle;
		*outputX = robotCurrentX + moveDistance * cos(currentAngle*3.1415926/180);
		*outputY = robotCurrentY - moveDistance * sin(currentAngle*3.1415926/180);
	}

}

void CoordinateCalculation(const double robotCurrentX,const double robotCurrentY,double *outputX,double *outputY,const double camAngle, const double robotHeading,const double movDistance)
{
	double originalX = robotCurrentX;
	double originalY = robotCurrentY;

	double angleRobot = robotHeading;
	double angleCam = camAngle;
	double currentAngle = angleRobot+angleCam;



	double targetX,targetY;

	targetX = originalX;

	double length = 0;
	double targetLength = movDistance;

	if(currentAngle > 180)
	{
		currentAngle -= 360;
	}
	else if(currentAngle < -180)
	{
		currentAngle += 360;
	}

	targetY = tan(currentAngle*PI/180)*(targetX - originalX) + originalY;
	length = sqrt((targetY - originalY)*(targetY - originalY) +(targetX - originalX)*(targetX - originalX));


	while(length < targetLength)
	{
		if(currentAngle == 0)
		{
			targetX += 1;
			length = abs(targetX - originalX);
			continue;
		}
		else if(currentAngle == 180 || currentAngle == -180)
		{
			targetX -=1;
			length = abs(targetX - originalX);
			continue;
		}
		else if(currentAngle == 90)
		{
			targetY += 1;
			length = abs(targetY - originalY);
			continue;
		}
		else if(currentAngle == -90)
		{
			targetY -= 1;
			length = abs(targetY - originalY);
			continue;	
		}
		else if(currentAngle< 90 && currentAngle >-90)
		{
			targetX += 1;
		}
		else if((currentAngle > 90 &&currentAngle < 180)||(currentAngle > -180 && currentAngle < -90))
		{
			targetX -= 1;
		}
		targetY = tan(currentAngle*PI/180)*(targetX - originalX) + originalY;
		length = sqrt((targetY - originalY)*(targetY - originalY) +(targetX - originalX)*(targetX - originalX));
		// 		cout << "Currently, the length from the original point is " << length << endl;
	}

	if(targetX >= 2500)
	{
		targetX = 2200;
	}
	if(targetX <= -840)
	{
		targetX = -830;
	}
	if(targetY >= 1200)
	{
		targetY = 1000;
	}
	if(targetY <= -3200)
	{
		targetY = -2300;
	}

	cout << "targetX is " << targetX << "targetY is " << targetY << endl;

	*outputX = targetX;
	*outputY = targetY;

	return;
}







//-------------------------Legacy---------------------------------------
// int Turnangle=1;

void turn_func( ArServerClient *serverclient, ArNetPacket *socket)
{



	//   G_PathPlanning->pathPlanToPose(ArPose(500,-500), true);
	//   while(G_PathPlanning->getState() != ArPathPlanningTask::REACHED_GOAL );
	//   serverclient->sendPacketTcp(socket);


	//Path Planning Add by YMZ--------------------------------------------------------------


	//-----------------------------------------------------------------------------


	//   ptrPathPlanning->cancelPathPlan();
	//   if(ptrPathPlanning->pathPlanToPose(ArPose(0,0,0),1))
	//     printf("Heading to Home!\n"); 
	//     
	//   else printf("The Home location is not exist!");
	//   robot.lock();
	//   robot.setHeading(90*Turnangle);
	//   Turnangle++;
	//   robot.unlock();
	//   start.setToNow();
	// 	while (1)
	// 	{
	// 		robot.lock();
	// 		if (robot.isHeadingDone())
	// 		{
	// 			printf("directMotionExample: Finished turn\n");
	// 			robot.unlock();
	// 			ArUtil::sleep(50);
	// 			serverclient->sendPacketTcp(&pkg);
	// 			break;
	// 		}
	// 		if (start.mSecSince() > 5000)
	// 		{
	// 			printf("directMotionExample: Turn timed out\n");
	// 			robot.unlock();
	// 			serverclient->sendPacketTcp(&pkg);
	// 			break;
	// 		}
	// 		robot.unlock();
	// 		ArUtil::sleep(10);
	// 	}

}



// int findNeighbor(int *goal)
// {
//   
//   double currentX = robot.getPose().getX();
//   double currentY = robot.getPose().getY();
//   int index = 0;  
//   
//   for(int i = 0; i < sizeof(goal);)
//   {
// 
//     double min = abs(currentX - goal[i++]) + abs(currentY - goal[i++]);
//     double max = abs(currentX - goal[i++]) + abs(currentY - goal[i++]);
//     
//     if(min <= max)
//     {
//       index = i - 3;
//     }
//     else
//     {
//       index = i - 1;
//     }
//   }
//   
//   return index;
// }
