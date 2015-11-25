/*
 * Copyright (c) 2012-2015 Aldebaran Robotics. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the COPYING file.
 */
/*
 * Codigo fue modificado para este caso
 */

//includes basicos
#include <iostream>
#include <string>
#include <stdlib.h>
#include <stdio.h>


// Aldebaran includes.
#include <alproxies/alvideodeviceproxy.h>
#include <alvision/alimage.h>
#include <alvision/alvisiondefinitions.h>
#include <alerror/alerror.h>

// Opencv includes.
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>

//

using namespace AL;
using namespace cv;
using namespace std;


//-------IMPORTANTE--------
//En ocasiones cuando hay muchas formas en la camara el programa truena,
//si comienza a fallar tratar con otro tipo de configuracion

//Seleccionar el tipo de calibracion
//calibracion 0, todas las ventanas
//calibracion 1, no sale la opcion de thresholds
//calibracion 2, no salen los contornos
//calibracion 3, no salen los thresholds ni los contornos.

int calibracion = 0;

/// Global variables

//Se crean los frames
Mat src, src1, erosion_dst, dilation_dst, both_dst, orig;

int posX, posY;
int erosion_elem = 0;
int erosion_size = 0;

//COPY

//////COPY/////////
int iLowH = 0;
int iHighH = 17;
int iLowS = 177;
int iHighS = 255;
int iLowV = 50;
int iHighV = 255;
int dilation_elem = 0;
int dilation_size = 1;
int thresh = 100;
////END COPY////////

//END

int const max_elem = 2;
int const max_kernel_size = 21;

 //Variables para los contornos
 Mat srcContorno; Mat src_gray;
 int max_thresh = 255;
 RNG rng(12345);
 int avgX = 0;
 int avgY = 0;
 int totalContorno = 0;
 //Fin contornos

/** Function Headers */
void Erosion( int, void* );
void Dilation( int, void* );

/// Function header
void thresh_callback(int, void* );


/**
* \brief Shows images retrieved from the robot.
*
* \param robotIp the IP adress of the robot
*/

int main(int argc, char* argv[])
{
  //Nos conectamos con el robot
  const std::string robotIp(argv[1]);
  try
  {
        /** Create a proxy to ALVideoDevice on the robot.*/
        ALVideoDeviceProxy camProxy(robotIp, 9559);

        /** Subscribe a client image requiring 320*240 and BGR colorspace.*/
        const std::string clientName = camProxy.subscribe("test", kQVGA, kBGRColorSpace, 30);
        camProxy.setActiveCamera(1); //conect to bottom camera
        camProxy.setResolution("test", 1);

        /** Create an cv::Mat header to wrap into an opencv image.*/
        cv::Mat imgHeader = cv::Mat(cv::Size(320, 240), CV_8UC3);

        /** Create a OpenCV window to display the images. */
       // cv::namedWindow("images");

        /** Main loop. Exit when pressing ESC.*/
        while ((char) cv::waitKey(5) != 27)
        {
          /** Retrieve an image from the camera.
          * The image is returned in the form of a container object, with the
          * following fields:
          * 0 = width
          * 1 = height
          * 2 = number of layers
          * 3 = colors space index (see alvisiondefinitions.h)
          * 4 = time stamp (seconds)
          * 5 = time stamp (micro seconds)
          * 6 = image buffer (size of width * height * number of layers)
          */
          ALValue img = camProxy.getImageRemote(clientName);

          /** Access the image buffer (6th field) and assign it to the opencv image
          * container. */
          imgHeader.data = (uchar*) img[6].GetBinary();

          /** Tells to ALVideoDevice that it can give back the image buffer to the
          * driver. Optional after a getImageRemote but MANDATORY after a getImageLocal.*/
          camProxy.releaseImage(clientName);



          /// Load an image
          ///
          // Clonamos las imagenes para poder procesarlas
          src =  imgHeader.clone();
          orig = imgHeader.clone();


         cvtColor(src, src, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV//Threshold the image

         //Codigo original del programa
         src1 = src;
         if( src.empty() ){
             return -1;
         }

           /// Create windows
           namedWindow( "Dilation Demo", WINDOW_AUTOSIZE );

           /// Create Dilation Trackbar
           createTrackbar( "iLowH", "Dilation Demo",
                   &iLowH, 255,
                   Dilation );
           createTrackbar( "iHighH", "Dilation Demo",
                   &iHighH, 255,
                   Dilation );
           createTrackbar( "iLowS", "Dilation Demo",
                   &iLowS, 255,
                   Dilation );
           createTrackbar( "iHighS", "Dilation Demo",
                   &iHighS, 255,
                   Dilation );
           createTrackbar( "iLowV", "Dilation Demo",
                   &iLowV, 255,
                   Dilation );
           createTrackbar( "iHighV", "Dilation Demo",
                   &iHighV, 255,
                   Dilation );
           createTrackbar( "dilation_elem", "Dilation Demo",
                   &dilation_elem, 2,
                   Dilation );
           createTrackbar( "Kernel size:\n 2n +1", "Dilation Demo",
                   &dilation_size, max_kernel_size,
                   Dilation );

           //////////////////////////////////////////////////////////////////////////////////////////

           //Dilatamos las imagenes
           Dilation( 0, 0 );

           //Seguimos la pelota de acuerdo al algoritmo orifinal

           // /*

           Moments oMoments = moments(dilation_dst);
           double dM01 = oMoments.m01;
           double dM10 = oMoments.m10;
           double dArea = oMoments.m00;

           //Calculamos la posicion de la pelota
           posX = dM10 / dArea;
           posY = dM01 / dArea;

           //Imprimimos la posicion
           if( posX > 0 && posY > 0)
              cout << "pos X = " << posX << " pos Y = " << posY << endl;
           //   */

           // Termina el codigo original para seguir la pelota


           /////Comienza el nuevo codigo

           //Moments oMoments = moments(dilation_dst);
           //double dArea = oMoments.m00;

            // /*
           // Clonamos los frames
           src_gray = dilation_dst.clone();

           // Convert image to gray and blur it
           blur( src_gray, src_gray, Size(3,3) );


           //Aparece dependiendo el tipo de calibracion
           //calibracion 0, todas las ventanas
           //calibracion 1, no sale la opcion de thresholds
           //calibracion 2, no salen los contornos
           //calibracion 3, no salen los thresholds ni los contornos.
           // Create Window and display
           if(calibracion == 0 || calibracion == 2){

              char* source_window = "Source";
              namedWindow( source_window, CV_WINDOW_AUTOSIZE );
              imshow( source_window, src_gray );

              //Llamamos a thresh_callback e imprimimos las variables originales
              createTrackbar( " Canny thresh:", "Source", &thresh, max_thresh, thresh_callback );
           }
           thresh_callback( 0, 0 );
           cout << "avg X = " << avgX << " avg Y = " << avgY << endl;

           //Circulo con el nuevo algoritmo
           Point pt = Point( avgX, avgY);
           if(avgX > 0 && avgX < 400 && avgY > 0 && avgY < 400){
             circle (orig, pt,sqrt(sqrt(dArea)), CV_RGB(255,0,0),5 );
           }
            // */

           /////Termina el nuevo codigo

           //Circulo con el algoritmo original
           Point pt2 = Point( posX, posY);
           if(posX > 0 && posX < 400 && posY > 0 && posY < 400){
             circle (orig, pt2,sqrt(sqrt(dArea)), CV_RGB(0,0,255),5 );
           }

           // Desplegamos las imagenes
           imshow( "Dilation Demo", dilation_dst );
           imshow( "orig", orig );
          // waitKey(50);

           //Imprimimos en consola las variables para pegarlas en el main del codigo kick_vision
          if( (char) waitKey(20) == 'p'||(char) waitKey(20) == 32){
              cout<<"//////COPY/////////"<<endl;
              cout<<"int iLowH = "<<iLowH<<";"<<endl;
              cout<<"int iHighH = "<<iHighH<<";"<<endl;
              cout<<"int iLowS = "<<iLowS<<";"<<endl;
              cout<<"int iHighS = "<<iHighS<<";"<<endl;
              cout<<"int iLowV = "<<iLowV<<";"<<endl;
              cout<<"int iHighV = "<<iHighV<<";"<<endl;
              cout<<"int dilation_elem = "<<dilation_elem<<";"<<endl;
              cout<<"int dilation_size = "<<dilation_size<<";"<<endl;
              cout<<"int thresh = "<<thresh<<";"<<endl;
              cout<<"////END COPY////////"<<endl;

          }

        }

        /** Cleanup.*/
        camProxy.unsubscribe(clientName);

  }
  catch (const AL::ALError& e)
  {
    std::cerr << "Caught exception " << e.what() << std::endl;
  }

    if (argc < 2)
    {
      std::cerr << "Usage 'getimages robotIp'" << std::endl;
      return 1;
    }

   return 0;

}

/**
 * @function Erosion
 */


/**
 * @function Dilation
 * Se dilata la imagen y se procesa con los terminos establecidos en los sliders
 */
void Dilation( int, void* )
{
  int dilation_type = 0;
  if( dilation_elem == 0 ){ dilation_type = MORPH_RECT; }
  else if( dilation_elem == 1 ){ dilation_type = MORPH_CROSS; }
  else if( dilation_elem == 2) { dilation_type = MORPH_ELLIPSE; }

inRange(src1, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), src);
  Mat element = getStructuringElement( dilation_type,
                       Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                       Point( dilation_size, dilation_size ) );

  dilate( src, dilation_dst, element );
}

//Codigo calcula y ubica los centroides de los contornos
void thresh_callback(int, void* )
{
  int sumX = 0;
  int sumY = 0;


  Mat canny_output;
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;

  /// Detect edges using canny
  Canny( src_gray, canny_output, thresh, thresh*2, 3 );
  /// Find contours
  findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

  /// Get the moments
  vector<Moments> mu(contours.size() );
  for( int i = 0; i < contours.size(); i++ )
     { mu[i] = moments( contours[i], false ); }

  ///  Get the mass centers:
  vector<Point2f> mc( contours.size() );
  for( int i = 0; i < contours.size(); i++ )
     { mc[i] = Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 ); }

  /// Draw contours
  Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
  for( int i = 0; i< contours.size(); i++ )
     {
       Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
       drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
       circle( drawing, mc[i], 4, color, -1, 8, 0 );
     }

    //Aparece dependiendo el tipo de calibracion
    //calibracion 0, todas las ventanas
    //calibracion 1, no sale la opcion de thresholds
    //calibracion 2, no salen los contornos
    //calibracion 3, no salen los thresholds ni los contornos.
    // Create Window and display
  if(calibracion == 0 || calibracion == 1){

      /// Show in a window
      namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
      imshow( "Contours", drawing );
  }

  /// Calculate the area with the moments 00 and compare with the result of the OpenCV function
  ///
  //printf("\t Info: Area and Contour Length \n");
  for( int i = 0; i< contours.size(); i++ )
     {

        //Se queda comentado para fines de debuging
        //imprime los valores que recivimos de los centroides
        /*
        printf(" * Contour[%d] - Area (M_00) = %.2f - Area OpenCV: %.2f - Length: %.2f \n", i, mu[i].m00, contourArea(contours[i]), arcLength( contours[i], true ) );
        Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
        drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
        circle( drawing, mc[i], 4, color, -1, 8, 0 );
        */

        //Calculamos el promedio de los centroides
        if(contourArea(contours[i]) >= 150){
        totalContorno++;
        sumX += mc[i].x;
        sumY += mc[i].y;
        }
     }

      avgX = (int) (sumX/totalContorno);
      avgY = (int) (sumY/totalContorno);
      sumX = 0;
      sumY = 0;
      totalContorno = 0;

  /*

  //Desplegamos el numero de contornos
  cout<<"Contornos ="<< contours.size()<<endl;

  //Verificacmos que algoritmo es el mejor

  if(totalContorno > 0 && totalContorno < 4){
      // asignamos el promedio y limpiamos las variables
      avgX = (int) (sumX/totalContorno);
      avgY = (int) (sumY/totalContorno);
      sumX = 0;
      sumY = 0;
      totalContorno = 0;
      cout<<"algoritomo nuevo"<<endl;

  }else{

      Moments oMoments = moments(dilation_dst);
      double dM01 = oMoments.m01;
      double dM10 = oMoments.m10;
      double dArea = oMoments.m00;

      //Calculamos la posicion de la pelota
      posX = dM10 / dArea;
      posY = dM01 / dArea;

       cout<<"algoritomo original"<<endl;
  }
  */

}
