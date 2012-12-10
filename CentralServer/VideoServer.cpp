



#include "VCCHandler.h"
#include "VideoServer.h"
#include "MotionsServer.h"
#define VIDEO_PORT 11000


ArSocket clientSock, serverSock;
ArMutex mutex_video;
int is_data_ready = 0;
Mat cap_img;
Mat gray_img;
//-------------------------------------------------------------------



class streamServer : public ArASyncTask
{
	void* runThread(void*)  
	{
		size_t strSize;


		// The socket objects: one for accepting new client connections,
		// and another for communicating with a client after it connects.


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
				//cout <<" Sending image to the client.grayImgSize= " << grayImgSize;
				if ((strSize = clientSock.write(gray_img.data, grayImgSize))==grayImgSize )

				is_data_ready = 0;
			}
			mutex_video.unlock();

			//if(!clientSock.isOpen())
			//{
			//	clientSock.close();
			//	ArLog::log(ArLog::Normal, " Socket to client closed.");
			//	if (serverSock.accept(&clientSock))
			//		ArLog::log(ArLog::Normal, " Client has connected.");
			//	else
			//		ArLog::log(ArLog::Terse, " Error in accepting a connection from the client: %s.", serverSock.getErrorStr().c_str());
			//}
		}
		//}

		//}
		// Now lets close the connection to the client
		clientSock.close();
		ArLog::log(ArLog::Normal, " Socket to client closed.");
		// And lets close the server port
		//serverSock.close();
		//ArLog::log(ArLog::Normal, " Server socket closed.");

		// Uninitialize Aria and exit the program
		//Aria::exit(0);

	} //end of runThread
};
streamServer ss;
void clientCloseCallback(ArServerClient * serverClient)
{
	//ss.cancelAll();
	clientSock.close();
	serverSock.close();
	

	resetMotion();
	//if(!ss.getRunning())
	ss.runAsync();

	cout << "client is closed, callback is done!" <<endl;

}

void VideoServerBase::RobotVideoCB (ArServerClient *serverclient, ArNetPacket* )
{
	//cout <<gray_img.rows << " " << gray_img.cols << " " <<  gray_img.type();

	ArNetPacket packet1,packet2;
	ArNetPacket *videoPacket;

	uchar *ptr;
	ptr=gray_img.data;
	int imgSizeRemainer = gray_img.rows*gray_img.cols;

	mutex_video.lock();
	while(imgSizeRemainer>packet1.MAX_DATA_LENGTH)
	{
		packet1.empty();
		//cout << imgSizeRemainer << endl;
		packet1.dataToBuf(ptr, packet1.MAX_DATA_LENGTH);
		ptr += packet1.MAX_DATA_LENGTH;

		imgSizeRemainer -= packet1.MAX_DATA_LENGTH;
		
		videoPacket = &packet1;
		serverclient->sendPacketTcp(videoPacket);
	}
	packet1.empty();

	packet1.dataToBuf(ptr, imgSizeRemainer);
	//cout << imgSizeRemainer << endl;
	videoPacket = &packet1;
	serverclient->sendPacketTcp(videoPacket);
	mutex_video.unlock();
	//cout << "send out image"<<endl;
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
		//mutex_video.lock();
		//
		//mutex_video.unlock();
		//imshow(" ",gray_img);
		//waitKey(1);


		mutex_video.lock();
		cvtColor(cap_img, gray_img, CV_BGR2GRAY);

		is_data_ready = 1;
		mutex_video.unlock();

	}

}
