#include "VideoServer.h"

#include "KeyPTZ.h"
#include "MotionsServer.h"
#define VIDEO_PORT 11000
//-----------------------Global Variables---------------------------
ArMutex mutex_video;
Mat cap_img;
Mat gray_img;
int is_data_ready = 0;
ArSocket clientSock, serverSock;

//-------------------------------------------------------------------



class streamServer : public ArASyncTask
{
	void* runThread(void*)  
	{
		size_t strSize;
		
		// Open the server socket
		if (serverSock.open(VIDEO_PORT, ArSocket::TCP))
			ArLog::log(ArLog::Normal, " Opened the server port %d.", VIDEO_PORT);
		else
			ArLog::log(ArLog::Normal, " Failed to open the server port: %s.", serverSock.getErrorStr().c_str());

		if (serverSock.accept(&clientSock))
			ArLog::log(ArLog::Normal, " Client has connected.");
		else
			ArLog::log(ArLog::Terse, " Error in accepting a connection from the client: %s.", serverSock.getErrorStr().c_str());
		
		while(1)
		{

					mutex_video.lock();
		
					if (is_data_ready) 
					{
						int grayImgSize=gray_img.rows*gray_img.cols;

						if ((strSize = clientSock.write(gray_img.data, grayImgSize))==grayImgSize )
							is_data_ready = 0;
					}
					mutex_video.unlock();

				}

		clientSock.close();
		ArLog::log(ArLog::Normal, " Socket to client closed.");


	} 
};

streamServer ss;
void clientCloseCallback(ArServerClient * serverClient)
{
	clientSock.close();
	serverSock.close();

	resetMotion();
	ss.runAsync();

	cout << "client is closed, callback is done!" <<endl;

}




void* VideoServerBase::runThread(void*) 
{
	VideoCapture capture(-1);

	capture.set( CV_CAP_PROP_FRAME_WIDTH, 640);
	capture.set( CV_CAP_PROP_FRAME_HEIGHT, 480);

	capture.read(cap_img);

	/* run the stream server as a separate thread */
	ss.runAsync();

	while(true) 
	{
		capture.read(cap_img);

		mutex_video.lock();
		cvtColor(cap_img, gray_img, CV_BGR2GRAY);

		//imshow(" ",gray_img);
		waitKey(1);

		is_data_ready = 1;
		mutex_video.unlock();

	}

}
