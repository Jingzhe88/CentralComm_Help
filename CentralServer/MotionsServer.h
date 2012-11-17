#ifndef ACTIONSERVER_H_H
#define ACTIONSERVER_H_H

#include "Aria.h"
#include "ArNetworking.h"
#include "Arnl.h"
#include "ArLocalizationTask.h"

#include <cmath>
#include <iostream>
using namespace std;
// using namespace cv;


void turn_func( ArServerClient *serverclient, ArNetPacket *socket);
void S_RobotMotion( ArServerClient *serverclient, ArNetPacket *socket);
void S_CameraMotion( ArServerClient *serverclient, ArNetPacket *socket);
void S_RobotTurnLeft( ArServerClient *serverclient, ArNetPacket *socket);
void S_RobotTurnRight( ArServerClient *serverclient, ArNetPacket *socket);
void S_TargetApproach_Obstacles( ArServerClient *serverclient, ArNetPacket *socket);
void S_TargetApproach( ArServerClient *serverclient, ArNetPacket *socket);
void S_GlassesCancel( ArServerClient *serverclient, ArNetPacket *socket);
void S_Calibration( ArServerClient *serverclient, ArNetPacket *socket);
double getAngle(double targetX, double targetY, double originalX, double originalY);
void CoordinateCalculation(const double robotCurrentX,const double robotCurrentY,double *outputX,double *outputY,const double camAngle, const double robotHeading,const double movDistance);
void coordinateCalculation(const double robotCurrentX,const double robotCurrentY,double *outputX,double *outputY,const double camAngle, const double robotHeading,const double movDistance);
#endif