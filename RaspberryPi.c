#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include "rs232.h"
#define BUF_SIZE 128

double D1 =30;
double W = 121;
double F = 68.5;
int main(int argc , char* argv[]){
  int i=0;
  int cport_nr=24; /* /dev/ttyUSB0 */
  int bdrate=57600; /* 9600 baud */

  char mode[]={'8','N','1',0}; // 8 data bits, no parity, 1 stop bit
  char str_send[2][BUF_SIZE]; // send data buffer
  unsigned char str_recv[BUF_SIZE]; // recv data buffer
  strcpy(str_send[0], "Marker 1 \n");
  strcpy(str_send[1], "Marker 2 \n");
  
  if(RS232_OpenComport(cport_nr, bdrate, mode))
  {
    printf("Can not open comport\n");
    return(0);
  }

  usleep(2000000);  /* waits 2000ms for stable condition */



    cv::Mat cameraMatrix, distCoeffs;
    cv::Mat markerImage;
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::aruco::drawMarker(dictionary,23,200,markerImage,1);
    
    cv::VideoCapture inputVideo;
    inputVideo.open(0);

    while (inputVideo.grab()){



	int n = RS232_PollComport(cport_nr, str_recv, (int)BUF_SIZE);
	if(n > 0){
      str_recv[n] = 0;   /* always put a "null" at the end of a string! */
      //printf("Received i bytes: 's'\n", n, (char *)str_recv);
	}
	i++;
    	


        cv::Mat image, imageCopy;
        inputVideo.retrieve(image);
        cv::resize(image,image,cv::Size(320,240));
        image.copyTo(imageCopy);

   	    std::vector<int> ids;
    	std::vector<std::vector<cv::Point2f> > corners;    cv::aruco::detectMarkers(image, dictionary, corners, ids);

	std::vector<cv::Point> corner;

    	if (ids.size() > 0)
    	{
        cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);
		double x1=corners[0][0].x;
		double y1=corners[0][0].y;
		double x2=corners[0][1].x;
		double y2=corners[0][1].y;
		
		double a1 = x2-x1;
		double b1 = y2-y1;
		
		double P = sqrt((a1*a1)+(b1*b1));
		double D = (F*W)/P;

		std::cout<<"Distance "<<D<<std::endl;
		
		if(D > 50)
		{
			RS232_cputs(cport_nr, str_send[1]); // sends string on serial
			usleep(200000);  /* waits for reply 100ms */			
		}

	}
    	cv::imshow("out", imageCopy);
    	char key = (char) cv::waitKey(27);
    	if (key == 27)
      	  break;
    }




    return 0;
}
