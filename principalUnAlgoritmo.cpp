/**
 * Editado y desarrollado por:
 * Alfredo Hinojosa
 * Enrique Hernandez
 * David Valles
 * Luis Reyna
 */

//Basic includes
#include <iostream>
#include <string>
#include <stdlib.h>
#include <stdio.h>

//Robot NAO includes
#include <alproxies/alrobotpostureproxy.h>
#include <alproxies/almotionproxy.h>
#include <alproxies/alvideodeviceproxy.h>
#include <alproxies/alledsproxy.h>
#include <alproxies/altexttospeechproxy.h>
#include <alvision/alimage.h>
#include <alvision/alvisiondefinitions.h>
#include <alproxies/allandmarkdetectionproxy.h>
#include <alproxies/almemoryproxy.h>
#include <alvision/alvisiondefinitions.h>
#include <alerror/alerror.h>

//OPEN CV includes
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>


using namespace std;
using namespace cv;
using namespace AL;

void robot1(AL::ALRobotPostureProxy posture, AL::ALMotionProxy motion, AL::ALVideoDeviceProxy camProxy, AL::ALLandMarkDetectionProxy naoMark, AL::ALMemoryProxy memProxy);
int getOrientation(vector<Point> &pts, Mat &img);

//GLOBALES

//AQUI se pega el codigo generado en el documento de Calibracion

//PASTE HERE

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

//END HERE


//Variables globales que se utilizan para calcular el centroide de acuerdo al contorno de las figuras
Mat srcContorno;
Mat src_gray;
int max_thresh = 255;
RNG rng(12345);

//Variables para sacar el centroide global
int avgX = 0;
int avgY = 0;
int totalContorno = 0;
//Fin contornos

//Variables de posicion XY del centroide originales
int posX = 0;
int posY = 0;

int checks = 0; // numero de veces que checa despues de hacer la patada, para saber a donde girar camara
bool celebra = false; // metio gol, moverse como cua

void thresh_callback(int, void* );


 int main( int argc, char** argv )
 {

     //Se conecta con el robot
     AL::ALRobotPostureProxy posture(argv[1], 9559);
     AL::ALVideoDeviceProxy camProxy(argv[1], 9559);
     AL::ALMotionProxy motion(argv[1], 9559);
     ALTextToSpeechProxy say(argv[1],9559);
     std::vector<std::string> names;
     AL::ALValue times, keys;
     const std::string clientName = camProxy.subscribe("test2", AL::kQVGA, AL::kBGRColorSpace, 30);

     bool flag = true;
     bool checking = false; // varible usada para saber si fue gol, para iniciar chequeo
     string postura;
     posture.goToPosture("StandInit", 0.5);

     //get robot Frame
     camProxy.setActiveCamera(1); //conect to bottom camera
     camProxy.setResolution("test2", 1);

     //Variables para los frames y colores
     int contFrames = 0;
     char key = 'a';

     //frame general con la imagen que recibimos del NAO
     Mat frame;
     bool robotDetectedIzq = false;
     bool robotDetectedDer = false;

     //frames que se modifican para filtrar los colores
     Mat imgHSV;
     Mat imgThresholded;

     //Acomodamos la cabeza del NAO para voltear a ver el piso
     motion.angleInterpolation("HeadPitch", (20.5*M_PI)/180, 1, true);

     //Avanzamos 7 centimetros hacia adelante
     motion.moveTo(0.07,0,0);

     //Continuamos hasta presionar la tecla ESC
     while (key != 27 && (flag || checking))
     {
         //Obtienes la imagen de la camara
         Mat imgHeader = cv::Mat(cv::Size(320, 240), CV_8UC3);

         //Obtenemos la imagen del nao

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
         imgHeader.data = (uchar*) img[6].GetBinary();
         camProxy.releaseImage(clientName);
         frame = imgHeader.clone();


         if(contFrames == 4) {

             //Variables del codigo original
             robotDetectedIzq = false;
             robotDetectedDer = false;
             contFrames = 0;

             //Convert the captured frame from BGR to HSV
             cvtColor(frame, imgHSV, COLOR_BGR2HSV);
             //Threshold the image
             inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded);

             //Esos rangos se modifican de acuerdo a las variables que recibimos del codigo de calibracion
             //Se utilizan para limpiar la imagen imgThresholded de cualquier ruido
             int dilation_type = 0;
             if( dilation_elem == 0 ){ dilation_type = MORPH_RECT; }
             else if( dilation_elem == 1 ){ dilation_type = MORPH_CROSS; }
             else if( dilation_elem == 2) { dilation_type = MORPH_ELLIPSE; }

             Mat element = getStructuringElement( dilation_type,
                                  Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                  Point( dilation_size, dilation_size ) );

             //Dilatamos la imagen
             dilate( imgThresholded, imgThresholded, element );


             //Codigo original con el que busca la pelota

             //----------------IMPORTANTE------------------
             //Cuidado donde se acaba el } del if mas abajo
              /*
             Moments oMoments = moments(imgThresholded);
             double dM01 = oMoments.m01;
             double dM10 = oMoments.m10;
             double dArea = oMoments.m00;

             ///PONER EL ULTIMO }
             if (dArea > 10000)
             {
                 //calculate the position of the ball
                 posX = dM10 / dArea;
                 posY = dM01 / dArea;

                 cout << "pos X = " << posX << " pos Y = " << posY << endl;

                 Point pt = Point( posX, posY);

                 circle (imgThresholded, pt, (posY/2), CV_RGB(255,0,0) );

                 imshow("Thresholded Image", imgThresholded); //show the thresholded image
                 imshow("Src", frame); //show the original image
                 waitKey(50);
             */
             //FIN ORIGINAL

             //NUEVOS PUNTOS

             //Obtenemos los momens
             Moments oMoments = moments(imgThresholded);

              //Recibimos el area del centroide
             double dArea = oMoments.m00;

              //Clonamos la imagen modificada para calcular los puntos
             src_gray = imgThresholded.clone();

             //Desenfocamos la imagen para suavizar los puntos
             blur( src_gray, src_gray, Size(3,3) );

             //imagen para debug
             //imshow( source_window, src_gray );

             //Llamamos al metodo thresh_callback
             if(!checking){
                 thresh_callback( 0, 0 );
             } else {
                 Moments oMoments = moments(imgThresholded);
                 double dM01 = oMoments.m01;
                 double dM10 = oMoments.m10;
                 double dArea = oMoments.m00;

                 //calculate the position of the ball
                 posX = dM10 / dArea;
                 posY = dM01 / dArea;
             }

             //Imprimimos la posicion nueva de x y Y
             cout << "avg X = " << avgX << " avg Y = " << avgY << endl;

             //Creamos un circulo y lo ponemos en la pantalla
             Point pt = Point( avgX, avgY);

             if(avgX > 0 && avgX < 400 && avgY > 0 && avgY < 400){
                circle (imgThresholded, pt,sqrt(sqrt(dArea)), CV_RGB(255,0,0),5 );
             }

             //Asignamos los valores del circulo a las variables originales del programa
             if(!checking){
                 posX = avgX;
                 posY = avgY;
             }

             //show the thresholded image
             imshow("Thresholded Image", imgThresholded);

             //show the original image
             imshow("Src", frame);
             waitKey(50);

             //FIN NUEVOS PUNTOS0

             if(checking){
                if(checks <= 4){
                    motion.angleInterpolation("HeadYaw", -0.4782, 1, true); // cabeza en posicion derecha
                } else if(checks <= 8){
                    motion.angleInterpolation("HeadYaw", 0, 1, true); // cabeza en posicion neutral
                } else if(checks > 9){
                    motion.angleInterpolation("HeadYaw", 0.4694, 1, true); // cabeza en posicion izq
                }
                checks++;
                if(posY < 140){
                    celebra = true;
                    checking = false;
                }
                if(checks == 13){
                    say.say("No no no");
                    checking == false;
                }
             }

             // AQUI VIENE LA CELEBRACION O TRISTEZA
             if(celebra){
                 // Movimiento de Cua
                 std::vector<std::string> names1;
                 AL::ALValue times1, keys1;
                  names1.reserve(23);
                  times1.arraySetSize(23);
                  keys1.arraySetSize(23);

                  names1.push_back("LAnklePitch");
                  times1[0].arraySetSize(2);
                  keys1[0].arraySetSize(2);

                  times1[0][0] = 0.933333;
                  keys1[0][0] = AL::ALValue::array(-0.121359, AL::ALValue::array(3, -0.311111, -0.00000), AL::ALValue::array(3, 0.688889, 0.00000));
                  times1[0][1] = 3.00000;
                  keys1[0][1] = AL::ALValue::array(-0.409751, AL::ALValue::array(3, -0.688889, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

                  names1.push_back("LAnkleRoll");
                  times1[1].arraySetSize(2);
                  keys1[1].arraySetSize(2);

                  times1[1][0] = 0.933333;
                  keys1[1][0] = AL::ALValue::array(0.0153604, AL::ALValue::array(3, -0.311111, -0.00000), AL::ALValue::array(3, 0.688889, 0.00000));
                  times1[1][1] = 3.00000;
                  keys1[1][1] = AL::ALValue::array(-0.116564, AL::ALValue::array(3, -0.688889, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

                  names1.push_back("LElbowRoll");
                  times1[2].arraySetSize(3);
                  keys1[2].arraySetSize(3);

                  times1[2][0] = 0.933333;
                  keys1[2][0] = AL::ALValue::array(-0.306757, AL::ALValue::array(3, -0.311111, -0.00000), AL::ALValue::array(3, 0.688889, 0.00000));
                  times1[2][1] = 3.00000;
                  keys1[2][1] = AL::ALValue::array(-1.30539, AL::ALValue::array(3, -0.688889, -0.00000), AL::ALValue::array(3, 0.911111, 0.00000));
                  times1[2][2] = 5.73333;
                  keys1[2][2] = AL::ALValue::array(-0.00872665, AL::ALValue::array(3, -0.911111, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

                  names1.push_back("LElbowYaw");
                  times1[3].arraySetSize(3);
                  keys1[3].arraySetSize(3);

                  times1[3][0] = 0.933333;
                  keys1[3][0] = AL::ALValue::array(-0.125830, AL::ALValue::array(3, -0.311111, -0.00000), AL::ALValue::array(3, 0.688889, 0.00000));
                  times1[3][1] = 3.00000;
                  keys1[3][1] = AL::ALValue::array(0.469363, AL::ALValue::array(3, -0.688889, -0.170355), AL::ALValue::array(3, 0.911111, 0.225308));
                  times1[3][2] = 5.73333;
                  keys1[3][2] = AL::ALValue::array(1.06116, AL::ALValue::array(3, -0.911111, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

                  names1.push_back("LHand");
                  times1[4].arraySetSize(3);
                  keys1[4].arraySetSize(3);

                  times1[4][0] = 0.933333;
                  keys1[4][0] = AL::ALValue::array(0.0161273, AL::ALValue::array(3, -0.311111, -0.00000), AL::ALValue::array(3, 0.688889, 0.00000));
                  times1[4][1] = 3.00000;
                  keys1[4][1] = AL::ALValue::array(0.0161273, AL::ALValue::array(3, -0.688889, -0.00000), AL::ALValue::array(3, 0.911111, 0.00000));
                  times1[4][2] = 5.73333;
                  keys1[4][2] = AL::ALValue::array(0.0174533, AL::ALValue::array(3, -0.911111, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

                  names1.push_back("LHipPitch");
                  times1[5].arraySetSize(2);
                  keys1[5].arraySetSize(2);

                  times1[5][0] = 0.933333;
                  keys1[5][0] = AL::ALValue::array(0.0749446, AL::ALValue::array(3, -0.311111, -0.00000), AL::ALValue::array(3, 0.688889, 0.00000));
                  times1[5][1] = 3.00000;
                  keys1[5][1] = AL::ALValue::array(-0.761086, AL::ALValue::array(3, -0.688889, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

                  names1.push_back("LHipRoll");
                  times1[6].arraySetSize(2);
                  keys1[6].arraySetSize(2);

                  times1[6][0] = 0.933333;
                  keys1[6][0] = AL::ALValue::array(-0.0477461, AL::ALValue::array(3, -0.311111, -0.00000), AL::ALValue::array(3, 0.688889, 0.00000));
                  times1[6][1] = 3.00000;
                  keys1[6][1] = AL::ALValue::array(0.0289540, AL::ALValue::array(3, -0.688889, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

                  names1.push_back("LHipYawPitch");
                  times1[7].arraySetSize(2);
                  keys1[7].arraySetSize(2);

                  times1[7][0] = 0.933333;
                  keys1[7][0] = AL::ALValue::array(0.0291025, AL::ALValue::array(3, -0.311111, -0.00000), AL::ALValue::array(3, 0.688889, 0.00000));
                  times1[7][1] = 3.00000;
                  keys1[7][1] = AL::ALValue::array(-0.319116, AL::ALValue::array(3, -0.688889, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

                  names1.push_back("LKneePitch");
                  times1[8].arraySetSize(2);
                  keys1[8].arraySetSize(2);

                  times1[8][0] = 0.933333;
                  keys1[8][0] = AL::ALValue::array(0.0855841, AL::ALValue::array(3, -0.311111, -0.00000), AL::ALValue::array(3, 0.688889, 0.00000));
                  times1[8][1] = 3.00000;
                  keys1[8][1] = AL::ALValue::array(1.04740, AL::ALValue::array(3, -0.688889, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

                  names1.push_back("LShoulderPitch");
                  times1[9].arraySetSize(3);
                  keys1[9].arraySetSize(3);

                  times1[9][0] = 0.933333;
                  keys1[9][0] = AL::ALValue::array(2.04170, AL::ALValue::array(3, -0.311111, -0.00000), AL::ALValue::array(3, 0.688889, 0.00000));
                  times1[9][1] = 3.00000;
                  keys1[9][1] = AL::ALValue::array(2.06319, AL::ALValue::array(3, -0.688889, -0.00000), AL::ALValue::array(3, 0.911111, 0.00000));
                  times1[9][2] = 5.73333;
                  keys1[9][2] = AL::ALValue::array(-0.781907, AL::ALValue::array(3, -0.911111, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

                  names1.push_back("LShoulderRoll");
                  times1[10].arraySetSize(3);
                  keys1[10].arraySetSize(3);

                  times1[10][0] = 0.933333;
                  keys1[10][0] = AL::ALValue::array(0.417205, AL::ALValue::array(3, -0.311111, -0.00000), AL::ALValue::array(3, 0.688889, 0.00000));
                  times1[10][1] = 3.00000;
                  keys1[10][1] = AL::ALValue::array(0.246933, AL::ALValue::array(3, -0.688889, -0.00000), AL::ALValue::array(3, 0.911111, 0.00000));
                  times1[10][2] = 5.73333;
                  keys1[10][2] = AL::ALValue::array(0.530580, AL::ALValue::array(3, -0.911111, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

                  names1.push_back("LWristYaw");
                  times1[11].arraySetSize(3);
                  keys1[11].arraySetSize(3);

                  times1[11][0] = 0.933333;
                  keys1[11][0] = AL::ALValue::array(-0.998676, AL::ALValue::array(3, -0.311111, -0.00000), AL::ALValue::array(3, 0.688889, 0.00000));
                  times1[11][1] = 3.00000;
                  keys1[11][1] = AL::ALValue::array(-1.01708, AL::ALValue::array(3, -0.688889, -0.00000), AL::ALValue::array(3, 0.911111, 0.00000));
                  times1[11][2] = 5.73333;
                  keys1[11][2] = AL::ALValue::array(-0.696386, AL::ALValue::array(3, -0.911111, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

                  names1.push_back("RAnklePitch");
                  times1[12].arraySetSize(2);
                  keys1[12].arraySetSize(2);

                  times1[12][0] = 0.933333;
                  keys1[12][0] = AL::ALValue::array(-0.102805, AL::ALValue::array(3, -0.311111, -0.00000), AL::ALValue::array(3, 0.688889, 0.00000));
                  times1[12][1] = 3.00000;
                  keys1[12][1] = AL::ALValue::array(-0.543063, AL::ALValue::array(3, -0.688889, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

                  names1.push_back("RAnkleRoll");
                  times1[13].arraySetSize(2);
                  keys1[13].arraySetSize(2);

                  times1[13][0] = 0.933333;
                  keys1[13][0] = AL::ALValue::array(-0.00456227, AL::ALValue::array(3, -0.311111, -0.00000), AL::ALValue::array(3, 0.688889, 0.00000));
                  times1[13][1] = 3.00000;
                  keys1[13][1] = AL::ALValue::array(0.135032, AL::ALValue::array(3, -0.688889, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

                  names1.push_back("RElbowRoll");
                  times1[14].arraySetSize(3);
                  keys1[14].arraySetSize(3);

                  times1[14][0] = 0.933333;
                  keys1[14][0] = AL::ALValue::array(0.435699, AL::ALValue::array(3, -0.311111, -0.00000), AL::ALValue::array(3, 0.688889, 0.00000));
                  times1[14][1] = 3.00000;
                  keys1[14][1] = AL::ALValue::array(1.27786, AL::ALValue::array(3, -0.688889, -0.151244), AL::ALValue::array(3, 0.777778, 0.170759));
                  times1[14][2] = 5.33333;
                  keys1[14][2] = AL::ALValue::array(1.44862, AL::ALValue::array(3, -0.777778, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

                  names1.push_back("RElbowYaw");
                  times1[15].arraySetSize(3);
                  keys1[15].arraySetSize(3);

                  times1[15][0] = 0.933333;
                  keys1[15][0] = AL::ALValue::array(0.222388, AL::ALValue::array(3, -0.311111, -0.00000), AL::ALValue::array(3, 0.688889, 0.00000));
                  times1[15][1] = 3.00000;
                  keys1[15][1] = AL::ALValue::array(0.374254, AL::ALValue::array(3, -0.688889, -0.0720258), AL::ALValue::array(3, 0.777778, 0.0813194));
                  times1[15][2] = 5.33333;
                  keys1[15][2] = AL::ALValue::array(0.682424, AL::ALValue::array(3, -0.777778, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

                  names1.push_back("RHand");
                  times1[16].arraySetSize(3);
                  keys1[16].arraySetSize(3);

                  times1[16][0] = 0.933333;
                  keys1[16][0] = AL::ALValue::array(0.0160194, AL::ALValue::array(3, -0.311111, -0.00000), AL::ALValue::array(3, 0.688889, 0.00000));
                  times1[16][1] = 3.00000;
                  keys1[16][1] = AL::ALValue::array(0.00818718, AL::ALValue::array(3, -0.688889, -0.00000), AL::ALValue::array(3, 0.777778, 0.00000));
                  times1[16][2] = 5.33333;
                  keys1[16][2] = AL::ALValue::array(0.0174533, AL::ALValue::array(3, -0.777778, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

                  names1.push_back("RHipPitch");
                  times1[17].arraySetSize(2);
                  keys1[17].arraySetSize(2);

                  times1[17][0] = 0.933333;
                  keys1[17][0] = AL::ALValue::array(0.0413610, AL::ALValue::array(3, -0.311111, -0.00000), AL::ALValue::array(3, 0.688889, 0.00000));
                  times1[17][1] = 3.00000;
                  keys1[17][1] = AL::ALValue::array(-0.653540, AL::ALValue::array(3, -0.688889, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

                  names1.push_back("RHipRoll");
                  times1[18].arraySetSize(2);
                  keys1[18].arraySetSize(2);

                  times1[18][0] = 0.933333;
                  keys1[18][0] = AL::ALValue::array(0.0168944, AL::ALValue::array(3, -0.311111, -0.00000), AL::ALValue::array(3, 0.688889, 0.00000));
                  times1[18][1] = 3.00000;
                  keys1[18][1] = AL::ALValue::array(-0.0444656, AL::ALValue::array(3, -0.688889, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

                  names1.push_back("RKneePitch");
                  times1[19].arraySetSize(2);
                  keys1[19].arraySetSize(2);

                  times1[19][0] = 0.933333;
                  keys1[19][0] = AL::ALValue::array(0.103898, AL::ALValue::array(3, -0.311111, -0.00000), AL::ALValue::array(3, 0.688889, 0.00000));
                  times1[19][1] = 3.00000;
                  keys1[19][1] = AL::ALValue::array(1.03657, AL::ALValue::array(3, -0.688889, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

                  names1.push_back("RShoulderPitch");
                  times1[20].arraySetSize(3);
                  keys1[20].arraySetSize(3);

                  times1[20][0] = 0.933333;
                  keys1[20][0] = AL::ALValue::array(1.06617, AL::ALValue::array(3, -0.311111, -0.00000), AL::ALValue::array(3, 0.688889, 0.00000));
                  times1[20][1] = 3.00000;
                  keys1[20][1] = AL::ALValue::array(0.943452, AL::ALValue::array(3, -0.688889, 0.122721), AL::ALValue::array(3, 0.777778, -0.138556));
                  times1[20][2] = 5.33333;
                  keys1[20][2] = AL::ALValue::array(-0.319395, AL::ALValue::array(3, -0.777778, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

                  names1.push_back("RShoulderRoll");
                  times1[21].arraySetSize(3);
                  keys1[21].arraySetSize(3);

                  times1[21][0] = 0.933333;
                  keys1[21][0] = AL::ALValue::array(-0.398883, AL::ALValue::array(3, -0.311111, -0.00000), AL::ALValue::array(3, 0.688889, 0.00000));
                  times1[21][1] = 3.00000;
                  keys1[21][1] = AL::ALValue::array(-0.0429939, AL::ALValue::array(3, -0.688889, -0.00000), AL::ALValue::array(3, 0.777778, 0.00000));
                  times1[21][2] = 5.33333;
                  keys1[21][2] = AL::ALValue::array(-0.307178, AL::ALValue::array(3, -0.777778, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

                  names1.push_back("RWristYaw");
                  times1[22].arraySetSize(3);
                  keys1[22].arraySetSize(3);

                  times1[22][0] = 0.933333;
                  keys1[22][0] = AL::ALValue::array(0.949504, AL::ALValue::array(3, -0.311111, -0.00000), AL::ALValue::array(3, 0.688889, 0.00000));
                  times1[22][1] = 3.00000;
                  keys1[22][1] = AL::ALValue::array(0.964844, AL::ALValue::array(3, -0.688889, -0.00000), AL::ALValue::array(3, 0.777778, 0.00000));
                  times1[22][2] = 5.33333;
                  keys1[22][2] = AL::ALValue::array(0.0331613, AL::ALValue::array(3, -0.777778, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

                  try {
                          motion.angleInterpolationBezier(names1, times1, keys1);
                       } catch(const std::exception&){

                           cout << "Error in celebration1 " << endl;
                       }

                  // Se sienta
                  posture.goToPosture("Sit",0.5);

                  // Movimiento de la mano - John Cena
                  std::vector<std::string> names2;
                  AL::ALValue times2, keys2;
                  names2.reserve(8);
                  times2.arraySetSize(8);
                  keys2.arraySetSize(8);

                  names2.push_back("LElbowRoll");
                  times2[0].arraySetSize(1);
                  keys2[0].arraySetSize(1);

                  times2[0][0] = 0.760000;
                  keys2[0][0] = AL::ALValue::array(-1.20108, AL::ALValue::array(3, -0.253333, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

                  names2.push_back("LElbowYaw");
                  times2[1].arraySetSize(1);
                  keys2[1].arraySetSize(1);

                  times2[1][0] = 0.760000;
                  keys2[1][0] = AL::ALValue::array(-0.440299, AL::ALValue::array(3, -0.253333, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

                  names2.push_back("LShoulderPitch");
                  times2[2].arraySetSize(1);
                  keys2[2].arraySetSize(1);

                  times2[2][0] = 0.760000;
                  keys2[2][0] = AL::ALValue::array(0.901949, AL::ALValue::array(3, -0.253333, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

                  names2.push_back("LShoulderRoll");
                  times2[3].arraySetSize(1);
                  keys2[3].arraySetSize(1);

                  times2[3][0] = 0.760000;
                  keys2[3][0] = AL::ALValue::array(0.243864, AL::ALValue::array(3, -0.253333, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

                  names2.push_back("RElbowRoll");
                  times2[4].arraySetSize(2);
                  keys2[4].arraySetSize(2);

                  times2[4][0] = 0.760000;
                  keys2[4][0] = AL::ALValue::array(1.24718, AL::ALValue::array(3, -0.253333, -0.00000), AL::ALValue::array(3, 0.213333, 0.00000));
                  times2[4][1] = 1.40000;
                  keys2[4][1] = AL::ALValue::array(1.56207, AL::ALValue::array(3, -0.213333, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

                  names2.push_back("RElbowYaw");
                  times2[5].arraySetSize(6);
                  keys2[5].arraySetSize(6);

                  times2[5][0] = 0.760000;
                  keys2[5][0] = AL::ALValue::array(0.500042, AL::ALValue::array(3, -0.253333, -0.00000), AL::ALValue::array(3, 0.213333, 0.00000));
                  times2[5][1] = 1.40000;
                  keys2[5][1] = AL::ALValue::array(-0.151844, AL::ALValue::array(3, -0.213333, -0.00000), AL::ALValue::array(3, 0.200000, 0.00000));
                  times2[5][2] = 2.00000;
                  keys2[5][2] = AL::ALValue::array(1.13795, AL::ALValue::array(3, -0.200000, -0.00000), AL::ALValue::array(3, 0.333333, 0.00000));
                  times2[5][3] = 3.00000;
                  keys2[5][3] = AL::ALValue::array(-0.265290, AL::ALValue::array(3, -0.333333, -0.00000), AL::ALValue::array(3, 0.266667, 0.00000));
                  times2[5][4] = 3.80000;
                  keys2[5][4] = AL::ALValue::array(1.06116, AL::ALValue::array(3, -0.266667, -0.00000), AL::ALValue::array(3, 0.266667, 0.00000));
                  times2[5][5] = 4.60000;
                  keys2[5][5] = AL::ALValue::array(-0.226893, AL::ALValue::array(3, -0.266667, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

                  names2.push_back("RShoulderPitch");
                  times2[6].arraySetSize(2);
                  keys2[6].arraySetSize(2);

                  times2[6][0] = 0.760000;
                  keys2[6][0] = AL::ALValue::array(-0.150290, AL::ALValue::array(3, -0.253333, -0.00000), AL::ALValue::array(3, 0.213333, 0.00000));
                  times2[6][1] = 1.40000;
                  keys2[6][1] = AL::ALValue::array(-0.174533, AL::ALValue::array(3, -0.213333, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

                  names2.push_back("RShoulderRoll");
                  times2[7].arraySetSize(1);
                  keys2[7].arraySetSize(1);

                  times2[7][0] = 0.760000;
                  keys2[7][0] = AL::ALValue::array(-0.282298, AL::ALValue::array(3, -0.253333, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

                  try {
                          motion.angleInterpolationBezier(names2, times2, keys2);
                       } catch(const std::exception&){

                           cout << "Error in celebration2 " << endl;
                       }

                 checking = false;

             } // Final checking


            //CODIGO ORGINAL PARA PATEAR EL BALON

             if(posX > 170 && !checking){
                 motion.moveTo(0, -0.02, 0);
                 motion.moveTo(0,0,(-10*M_PI/180));
             }
             if(posX < 150 && !checking){ // 170 c y s, 175 a.
                 motion.moveTo(0, 0.02, 0);
             }
             if(posY < 160 && !checking) { // catarino y sheldon con 165, lisa con 160
                 motion.moveTo(0.02,0,0); // catarino y sheldon con .05 y lisa con .04
             }
             if((posY> 160 && posX <= 170 && posX >= 150) && !checking){
                 say.setVolume(1);
                 say.say("KICK");

                 motion.moveTo(0.0,0.03,0);
                 motion.moveTo(0,0.0,(-5*M_PI/180));
                 names.reserve(22);
                 times.arraySetSize(22);
                 keys.arraySetSize(22);
                 names.push_back("HeadPitch");
                 times[0].arraySetSize(5);
                 keys[0].arraySetSize(5);

                 times[0][0] = 0.440000;
                 keys[0][0] = AL::ALValue::array(0.0245020, AL::ALValue::array(3, -0.146667, -0.00000), AL::ALValue::array(3, 0.253333, 0.00000));
                 times[0][1] = 1.20000;
                 keys[0][1] = AL::ALValue::array(0.0214340, AL::ALValue::array(3, -0.253333, -0.00000), AL::ALValue::array(3, 0.253333, 0.00000));
                 times[0][2] = 1.96000;
                 keys[0][2] = AL::ALValue::array(0.0229680, AL::ALValue::array(3, -0.253333, -0.00000), AL::ALValue::array(3, 0.280000, 0.00000));
                 times[0][3] = 2.80000;
                 keys[0][3] = AL::ALValue::array(0.0214340, AL::ALValue::array(3, -0.280000, -0.00000), AL::ALValue::array(3, 0.0800000, 0.00000));
                 times[0][4] = 3.04000;
                 keys[0][4] = AL::ALValue::array(0.0229680, AL::ALValue::array(3, -0.0800000, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

                 names.push_back("HeadYaw");
                 times[1].arraySetSize(5);
                 keys[1].arraySetSize(5);

                 times[1][0] = 0.440000;
                 keys[1][0] = AL::ALValue::array(-0.00617796, AL::ALValue::array(3, -0.146667, -0.00000), AL::ALValue::array(3, 0.253333, 0.00000));
                 times[1][1] = 1.20000;
                 keys[1][1] = AL::ALValue::array(-0.00924597, AL::ALValue::array(3, -0.253333, 0.000766999), AL::ALValue::array(3, 0.253333, -0.000766999));
                 times[1][2] = 1.96000;
                 keys[1][2] = AL::ALValue::array(-0.0107800, AL::ALValue::array(3, -0.253333, -0.00000), AL::ALValue::array(3, 0.280000, 0.00000));
                 times[1][3] = 2.80000;
                 keys[1][3] = AL::ALValue::array(-0.00310997, AL::ALValue::array(3, -0.280000, -0.00000), AL::ALValue::array(3, 0.0800000, 0.00000));
                 times[1][4] = 3.04000;
                 keys[1][4] = AL::ALValue::array(-0.0107800, AL::ALValue::array(3, -0.0800000, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

                 names.push_back("LAnklePitch");
                 times[2].arraySetSize(5);
                 keys[2].arraySetSize(5);

                 times[2][0] = 0.440000;
                 keys[2][0] = AL::ALValue::array(-0.408086, AL::ALValue::array(3, -0.146667, -0.00000), AL::ALValue::array(3, 0.253333, 0.00000));
                 times[2][1] = 1.20000;
                 keys[2][1] = AL::ALValue::array(-0.567621, AL::ALValue::array(3, -0.253333, -0.00000), AL::ALValue::array(3, 0.253333, 0.00000));
                 times[2][2] = 1.96000;
                 keys[2][2] = AL::ALValue::array(-0.567621, AL::ALValue::array(3, -0.253333, -0.00000), AL::ALValue::array(3, 0.280000, 0.00000));
                 times[2][3] = 2.80000;
                 keys[2][3] = AL::ALValue::array(-0.567621, AL::ALValue::array(3, -0.280000, -0.00000), AL::ALValue::array(3, 0.0800000, 0.00000));
                 times[2][4] = 3.04000;
                 keys[2][4] = AL::ALValue::array(-0.576826, AL::ALValue::array(3, -0.0800000, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

                 names.push_back("LAnkleRoll");
                 times[3].arraySetSize(5);
                 keys[3].arraySetSize(5);

                 times[3][0] = 0.440000;
                 keys[3][0] = AL::ALValue::array(-0.00302603, AL::ALValue::array(3, -0.146667, -0.00000), AL::ALValue::array(3, 0.253333, 0.00000));
                 times[3][1] = 1.20000;
                 keys[3][1] = AL::ALValue::array(0.0521979, AL::ALValue::array(3, -0.253333, -0.00000), AL::ALValue::array(3, 0.253333, 0.00000));
                 times[3][2] = 1.96000;
                 keys[3][2] = AL::ALValue::array(0.0521979, AL::ALValue::array(3, -0.253333, -0.00000), AL::ALValue::array(3, 0.280000, 0.00000));
                 times[3][3] = 2.80000;
                 keys[3][3] = AL::ALValue::array(0.0521979, AL::ALValue::array(3, -0.280000, -0.00000), AL::ALValue::array(3, 0.0800000, 0.00000));
                 times[3][4] = 3.04000;
                 keys[3][4] = AL::ALValue::array(0.0521980, AL::ALValue::array(3, -0.0800000, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

                 names.push_back("LElbowRoll");
                 times[4].arraySetSize(5);
                 keys[4].arraySetSize(5);

                 times[4][0] = 0.440000;
                 keys[4][0] = AL::ALValue::array(-0.00872665, AL::ALValue::array(3, -0.146667, -0.00000), AL::ALValue::array(3, 0.253333, 0.00000));
                 times[4][1] = 1.20000;
                 keys[4][1] = AL::ALValue::array(-0.0306380, AL::ALValue::array(3, -0.253333, 0.00441891), AL::ALValue::array(3, 0.253333, -0.00441891));
                 times[4][2] = 1.96000;
                 keys[4][2] = AL::ALValue::array(-0.0352401, AL::ALValue::array(3, -0.253333, -0.00000), AL::ALValue::array(3, 0.280000, 0.00000));
                 times[4][3] = 2.80000;
                 keys[4][3] = AL::ALValue::array(-0.0306380, AL::ALValue::array(3, -0.280000, -0.00000), AL::ALValue::array(3, 0.0800000, 0.00000));
                 times[4][4] = 3.04000;
                 keys[4][4] = AL::ALValue::array(-0.0352400, AL::ALValue::array(3, -0.0800000, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

                 names.push_back("LElbowYaw");
                 times[5].arraySetSize(5);
                 keys[5].arraySetSize(5);

                 times[5][0] = 0.440000;
                 keys[5][0] = AL::ALValue::array(-1.37451, AL::ALValue::array(3, -0.146667, -0.00000), AL::ALValue::array(3, 0.253333, 0.00000));
                 times[5][1] = 1.20000;
                 keys[5][1] = AL::ALValue::array(-1.37757, AL::ALValue::array(3, -0.253333, 0.00102276), AL::ALValue::array(3, 0.253333, -0.00102276));
                 times[5][2] = 1.96000;
                 keys[5][2] = AL::ALValue::array(-1.38064, AL::ALValue::array(3, -0.253333, 0.000728725), AL::ALValue::array(3, 0.280000, -0.000805433));
                 times[5][3] = 2.80000;
                 keys[5][3] = AL::ALValue::array(-1.38218, AL::ALValue::array(3, -0.280000, -0.00000), AL::ALValue::array(3, 0.0800000, 0.00000));
                 times[5][4] = 3.04000;
                 keys[5][4] = AL::ALValue::array(-1.38064, AL::ALValue::array(3, -0.0800000, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

                 names.push_back("LHipPitch");
                 times[6].arraySetSize(5);
                 keys[6].arraySetSize(5);

                 times[6][0] = 0.440000;
                 keys[6][0] = AL::ALValue::array(-0.464760, AL::ALValue::array(3, -0.146667, -0.00000), AL::ALValue::array(3, 0.253333, 0.00000));
                 times[6][1] = 1.20000;
                 keys[6][1] = AL::ALValue::array(-0.618161, AL::ALValue::array(3, -0.253333, -0.00000), AL::ALValue::array(3, 0.253333, 0.00000));
                 times[6][2] = 1.96000;
                 keys[6][2] = AL::ALValue::array(-0.616627, AL::ALValue::array(3, -0.253333, -0.000485813), AL::ALValue::array(3, 0.280000, 0.000536952));
                 times[6][3] = 2.80000;
                 keys[6][3] = AL::ALValue::array(-0.615092, AL::ALValue::array(3, -0.280000, -0.00000), AL::ALValue::array(3, 0.0800000, 0.00000));
                 times[6][4] = 3.04000;
                 keys[6][4] = AL::ALValue::array(-0.622762, AL::ALValue::array(3, -0.0800000, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

                 names.push_back("LHipRoll");
                 times[7].arraySetSize(5);
                 keys[7].arraySetSize(5);

                 times[7][0] = 0.440000;
                 keys[7][0] = AL::ALValue::array(4.19617e-05, AL::ALValue::array(3, -0.146667, -0.00000), AL::ALValue::array(3, 0.253333, 0.00000));
                 times[7][1] = 1.20000;
                 keys[7][1] = AL::ALValue::array(0.493989, AL::ALValue::array(3, -0.253333, -0.00000), AL::ALValue::array(3, 0.253333, 0.00000));
                 times[7][2] = 1.96000;
                 keys[7][2] = AL::ALValue::array(0.493989, AL::ALValue::array(3, -0.253333, -0.00000), AL::ALValue::array(3, 0.280000, 0.00000));
                 times[7][3] = 2.80000;
                 keys[7][3] = AL::ALValue::array(0.492455, AL::ALValue::array(3, -0.280000, -0.00000), AL::ALValue::array(3, 0.0800000, 0.00000));
                 times[7][4] = 3.04000;
                 keys[7][4] = AL::ALValue::array(0.493990, AL::ALValue::array(3, -0.0800000, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

                 names.push_back("LHipYawPitch");
                 times[8].arraySetSize(5);
                 keys[8].arraySetSize(5);

                 times[8][0] = 0.440000;
                 keys[8][0] = AL::ALValue::array(-0.00456004, AL::ALValue::array(3, -0.146667, -0.00000), AL::ALValue::array(3, 0.253333, 0.00000));
                 times[8][1] = 1.20000;
                 keys[8][1] = AL::ALValue::array(0.0614019, AL::ALValue::array(3, -0.253333, -0.00000), AL::ALValue::array(3, 0.253333, 0.00000));
                 times[8][2] = 1.96000;
                 keys[8][2] = AL::ALValue::array(0.0614019, AL::ALValue::array(3, -0.253333, -0.00000), AL::ALValue::array(3, 0.280000, 0.00000));
                 times[8][3] = 2.80000;
                 keys[8][3] = AL::ALValue::array(0.0614019, AL::ALValue::array(3, -0.280000, -0.00000), AL::ALValue::array(3, 0.0800000, 0.00000));
                 times[8][4] = 3.04000;
                 keys[8][4] = AL::ALValue::array(0.0614020, AL::ALValue::array(3, -0.0800000, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

                 names.push_back("LKneePitch");
                 times[9].arraySetSize(5);
                 keys[9].arraySetSize(5);

                 times[9][0] = 0.440000;
                 keys[9][0] = AL::ALValue::array(0.794570, AL::ALValue::array(3, -0.146667, -0.00000), AL::ALValue::array(3, 0.253333, 0.00000));
                 times[9][1] = 1.20000;
                 keys[9][1] = AL::ALValue::array(1.04001, AL::ALValue::array(3, -0.253333, -0.00460241), AL::ALValue::array(3, 0.253333, 0.00460241));
                 times[9][2] = 1.96000;
                 keys[9][2] = AL::ALValue::array(1.04461, AL::ALValue::array(3, -0.253333, -0.00145717), AL::ALValue::array(3, 0.280000, 0.00161055));
                 times[9][3] = 2.80000;
                 keys[9][3] = AL::ALValue::array(1.04921, AL::ALValue::array(3, -0.280000, -0.00000), AL::ALValue::array(3, 0.0800000, 0.00000));
                 times[9][4] = 3.04000;
                 keys[9][4] = AL::ALValue::array(1.04768, AL::ALValue::array(3, -0.0800000, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

                 names.push_back("LShoulderPitch");
                 times[10].arraySetSize(5);
                 keys[10].arraySetSize(5);

                 times[10][0] = 0.440000;
                 keys[10][0] = AL::ALValue::array(1.63213, AL::ALValue::array(3, -0.146667, -0.00000), AL::ALValue::array(3, 0.253333, 0.00000));
                 times[10][1] = 1.20000;
                 keys[10][1] = AL::ALValue::array(1.62600, AL::ALValue::array(3, -0.253333, 0.00357938), AL::ALValue::array(3, 0.253333, -0.00357938));
                 times[10][2] = 1.96000;
                 keys[10][2] = AL::ALValue::array(1.61066, AL::ALValue::array(3, -0.253333, -0.00000), AL::ALValue::array(3, 0.280000, 0.00000));
                 times[10][3] = 2.80000;
                 keys[10][3] = AL::ALValue::array(1.61526, AL::ALValue::array(3, -0.280000, -0.00000), AL::ALValue::array(3, 0.0800000, 0.00000));
                 times[10][4] = 3.04000;
                 keys[10][4] = AL::ALValue::array(1.60299, AL::ALValue::array(3, -0.0800000, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

                 names.push_back("LShoulderRoll");
                 times[11].arraySetSize(5);
                 keys[11].arraySetSize(5);

                 times[11][0] = 0.440000;
                 keys[11][0] = AL::ALValue::array(0.601287, AL::ALValue::array(3, -0.146667, -0.00000), AL::ALValue::array(3, 0.253333, 0.00000));
                 times[11][1] = 1.20000;
                 keys[11][1] = AL::ALValue::array(0.592082, AL::ALValue::array(3, -0.253333, 0.00383506), AL::ALValue::array(3, 0.253333, -0.00383506));
                 times[11][2] = 1.96000;
                 keys[11][2] = AL::ALValue::array(0.578276, AL::ALValue::array(3, -0.253333, -0.00000), AL::ALValue::array(3, 0.280000, 0.00000));
                 times[11][3] = 2.80000;
                 keys[11][3] = AL::ALValue::array(0.578276, AL::ALValue::array(3, -0.280000, -0.00000), AL::ALValue::array(3, 0.0800000, 0.00000));
                 times[11][4] = 3.04000;
                 keys[11][4] = AL::ALValue::array(0.576742, AL::ALValue::array(3, -0.0800000, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

                 names.push_back("RAnklePitch");
                 times[12].arraySetSize(5);
                 keys[12].arraySetSize(5);

                 times[12][0] = 0.440000;
                 keys[12][0] = AL::ALValue::array(-0.408002, AL::ALValue::array(3, -0.146667, -0.00000), AL::ALValue::array(3, 0.253333, 0.00000));
                 times[12][1] = 1.20000;
                 keys[12][1] = AL::ALValue::array(-0.0321720, AL::ALValue::array(3, -0.253333, -0.00000), AL::ALValue::array(3, 0.253333, 0.00000));
                 times[12][2] = 1.96000;
                 keys[12][2] = AL::ALValue::array(-0.880473, AL::ALValue::array(3, -0.253333, 0.0902144), AL::ALValue::array(3, 0.280000, -0.0997107));
                 times[12][3] = 2.80000;
                 keys[12][3] = AL::ALValue::array(-0.980184, AL::ALValue::array(3, -0.280000, -0.00000), AL::ALValue::array(3, 0.0800000, 0.00000));
                 times[12][4] = 3.04000;
                 keys[12][4] = AL::ALValue::array(0.360532, AL::ALValue::array(3, -0.0800000, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

                 names.push_back("RAnkleRoll");
                 times[13].arraySetSize(5);
                 keys[13].arraySetSize(5);

                 times[13][0] = 0.440000;
                 keys[13][0] = AL::ALValue::array(0.00464395, AL::ALValue::array(3, -0.146667, -0.00000), AL::ALValue::array(3, 0.253333, 0.00000));
                 times[13][1] = 1.20000;
                 keys[13][1] = AL::ALValue::array(0.380475, AL::ALValue::array(3, -0.253333, -0.00000), AL::ALValue::array(3, 0.253333, 0.00000));
                 times[13][2] = 1.96000;
                 keys[13][2] = AL::ALValue::array(0.165714, AL::ALValue::array(3, -0.253333, 0.0333097), AL::ALValue::array(3, 0.280000, -0.0368160));
                 times[13][3] = 2.80000;
                 keys[13][3] = AL::ALValue::array(0.128898, AL::ALValue::array(3, -0.280000, 0.0119311), AL::ALValue::array(3, 0.0800000, -0.00340889));
                 times[13][4] = 3.04000;
                 keys[13][4] = AL::ALValue::array(0.119694, AL::ALValue::array(3, -0.0800000, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

                 names.push_back("RElbowRoll");
                 times[14].arraySetSize(5);
                 keys[14].arraySetSize(5);

                 times[14][0] = 0.440000;
                 keys[14][0] = AL::ALValue::array(0.00872665, AL::ALValue::array(3, -0.146667, -0.00000), AL::ALValue::array(3, 0.253333, 0.00000));
                 times[14][1] = 1.20000;
                 keys[14][1] = AL::ALValue::array(0.0245859, AL::ALValue::array(3, -0.253333, -0.00306812), AL::ALValue::array(3, 0.253333, 0.00306812));
                 times[14][2] = 1.96000;
                 keys[14][2] = AL::ALValue::array(0.0276540, AL::ALValue::array(3, -0.253333, -0.000971542), AL::ALValue::array(3, 0.280000, 0.00107381));
                 times[14][3] = 2.80000;
                 keys[14][3] = AL::ALValue::array(0.0307220, AL::ALValue::array(3, -0.280000, -0.00000), AL::ALValue::array(3, 0.0800000, 0.00000));
                 times[14][4] = 3.04000;
                 keys[14][4] = AL::ALValue::array(0.0291880, AL::ALValue::array(3, -0.0800000, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

                 names.push_back("RElbowYaw");
                 times[15].arraySetSize(5);
                 keys[15].arraySetSize(5);

                 times[15][0] = 0.440000;
                 keys[15][0] = AL::ALValue::array(1.40817, AL::ALValue::array(3, -0.146667, -0.00000), AL::ALValue::array(3, 0.253333, 0.00000));
                 times[15][1] = 1.20000;
                 keys[15][1] = AL::ALValue::array(1.40203, AL::ALValue::array(3, -0.253333, -0.00000), AL::ALValue::array(3, 0.253333, 0.00000));
                 times[15][2] = 1.96000;
                 keys[15][2] = AL::ALValue::array(1.40510, AL::ALValue::array(3, -0.253333, -0.00000), AL::ALValue::array(3, 0.280000, 0.00000));
                 times[15][3] = 2.80000;
                 keys[15][3] = AL::ALValue::array(1.40357, AL::ALValue::array(3, -0.280000, 0.00119303), AL::ALValue::array(3, 0.0800000, -0.000340865));
                 times[15][4] = 3.04000;
                 keys[15][4] = AL::ALValue::array(1.40050, AL::ALValue::array(3, -0.0800000, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

                 names.push_back("RHipPitch");
                 times[16].arraySetSize(5);
                 keys[16].arraySetSize(5);

                 times[16][0] = 0.440000;
                 keys[16][0] = AL::ALValue::array(-0.466378, AL::ALValue::array(3, -0.146667, -0.00000), AL::ALValue::array(3, 0.253333, 0.00000));
                 times[16][1] = 1.20000;
                 keys[16][1] = AL::ALValue::array(-0.156510, AL::ALValue::array(3, -0.253333, -0.0871824), AL::ALValue::array(3, 0.253333, 0.0871824));
                 times[16][2] = 1.96000;
                 keys[16][2] = AL::ALValue::array(0.0567160, AL::ALValue::array(3, -0.253333, -0.00000), AL::ALValue::array(3, 0.280000, 0.00000));
                 times[16][3] = 2.80000;
                 keys[16][3] = AL::ALValue::array(0.00302603, AL::ALValue::array(3, -0.280000, 0.0536900), AL::ALValue::array(3, 0.0800000, -0.0153400));
                 times[16][4] = 3.04000;
                 keys[16][4] = AL::ALValue::array(-0.699546, AL::ALValue::array(3, -0.0800000, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

                 names.push_back("RHipRoll");
                 times[17].arraySetSize(5);
                 keys[17].arraySetSize(5);

                 times[17][0] = 0.440000;
                 keys[17][0] = AL::ALValue::array(-0.00456004, AL::ALValue::array(3, -0.146667, -0.00000), AL::ALValue::array(3, 0.253333, 0.00000));
                 times[17][1] = 1.20000;
                 keys[17][1] = AL::ALValue::array(0.282298, AL::ALValue::array(3, -0.253333, -0.00000), AL::ALValue::array(3, 0.253333, 0.00000));
                 times[17][2] = 1.96000;
                 keys[17][2] = AL::ALValue::array(0.257754, AL::ALValue::array(3, -0.253333, 0.0119013), AL::ALValue::array(3, 0.280000, -0.0131541));
                 times[17][3] = 2.80000;
                 keys[17][3] = AL::ALValue::array(0.207132, AL::ALValue::array(3, -0.280000, -0.00000), AL::ALValue::array(3, 0.0800000, 0.00000));
                 times[17][4] = 3.04000;
                 keys[17][4] = AL::ALValue::array(0.280764, AL::ALValue::array(3, -0.0800000, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

                 names.push_back("RHipYawPitch");
                 times[18].arraySetSize(5);
                 keys[18].arraySetSize(5);

                 times[18][0] = 0.440000;
                 keys[18][0] = AL::ALValue::array(-0.00456004, AL::ALValue::array(3, -0.146667, -0.00000), AL::ALValue::array(3, 0.253333, 0.00000));
                 times[18][1] = 1.20000;
                 keys[18][1] = AL::ALValue::array(0.0614019, AL::ALValue::array(3, -0.253333, -0.00000), AL::ALValue::array(3, 0.253333, 0.00000));
                 times[18][2] = 1.96000;
                 keys[18][2] = AL::ALValue::array(0.0614019, AL::ALValue::array(3, -0.253333, -0.00000), AL::ALValue::array(3, 0.280000, 0.00000));
                 times[18][3] = 2.80000;
                 keys[18][3] = AL::ALValue::array(0.0614019, AL::ALValue::array(3, -0.280000, -0.00000), AL::ALValue::array(3, 0.0800000, 0.00000));
                 times[18][4] = 3.04000;
                 keys[18][4] = AL::ALValue::array(0.0614020, AL::ALValue::array(3, -0.0800000, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

                 names.push_back("RKneePitch");
                 times[19].arraySetSize(5);
                 keys[19].arraySetSize(5);

                 times[19][0] = 0.440000;
                 keys[19][0] = AL::ALValue::array(0.797722, AL::ALValue::array(3, -0.146667, -0.00000), AL::ALValue::array(3, 0.253333, 0.00000));
                 times[19][1] = 1.20000;
                 keys[19][1] = AL::ALValue::array(-0.00302603, AL::ALValue::array(3, -0.253333, -0.00000), AL::ALValue::array(3, 0.253333, 0.00000));
                 times[19][2] = 1.96000;
                 keys[19][2] = AL::ALValue::array(0.859083, AL::ALValue::array(3, -0.253333, -0.279802), AL::ALValue::array(3, 0.280000, 0.309255));
                 times[19][3] = 2.80000;
                 keys[19][3] = AL::ALValue::array(1.76414, AL::ALValue::array(3, -0.280000, -0.00000), AL::ALValue::array(3, 0.0800000, 0.00000));
                 times[19][4] = 3.04000;
                 keys[19][4] = AL::ALValue::array(-0.0923279, AL::ALValue::array(3, -0.0800000, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

                 names.push_back("RShoulderPitch");
                 times[20].arraySetSize(5);
                 keys[20].arraySetSize(5);

                 times[20][0] = 0.440000;
                 keys[20][0] = AL::ALValue::array(1.52944, AL::ALValue::array(3, -0.146667, -0.00000), AL::ALValue::array(3, 0.253333, 0.00000));
                 times[20][1] = 1.20000;
                 keys[20][1] = AL::ALValue::array(1.50950, AL::ALValue::array(3, -0.253333, -0.00000), AL::ALValue::array(3, 0.253333, 0.00000));
                 times[20][2] = 1.96000;
                 keys[20][2] = AL::ALValue::array(1.51103, AL::ALValue::array(3, -0.253333, -0.000728451), AL::ALValue::array(3, 0.280000, 0.000805130));
                 times[20][3] = 2.80000;
                 keys[20][3] = AL::ALValue::array(1.51410, AL::ALValue::array(3, -0.280000, -0.00000), AL::ALValue::array(3, 0.0800000, 0.00000));
                 times[20][4] = 3.04000;
                 keys[20][4] = AL::ALValue::array(1.50643, AL::ALValue::array(3, -0.0800000, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

                 names.push_back("RShoulderRoll");
                 times[21].arraySetSize(5);
                 keys[21].arraySetSize(5);

                 times[21][0] = 0.440000;
                 keys[21][0] = AL::ALValue::array(-0.779314, AL::ALValue::array(3, -0.146667, -0.00000), AL::ALValue::array(3, 0.253333, 0.00000));
                 times[21][1] = 1.20000;
                 keys[21][1] = AL::ALValue::array(-0.739430, AL::ALValue::array(3, -0.253333, -0.00613479), AL::ALValue::array(3, 0.253333, 0.00613479));
                 times[21][2] = 1.96000;
                 keys[21][2] = AL::ALValue::array(-0.733295, AL::ALValue::array(3, -0.253333, -0.00170007), AL::ALValue::array(3, 0.280000, 0.00187902));
                 times[21][3] = 2.80000;
                 keys[21][3] = AL::ALValue::array(-0.728692, AL::ALValue::array(3, -0.280000, -0.00238645), AL::ALValue::array(3, 0.0800000, 0.000681843));
                 times[21][4] = 3.04000;
                 keys[21][4] = AL::ALValue::array(-0.724090, AL::ALValue::array(3, -0.0800000, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

                 try {
                       motion.angleInterpolationBezier(names, times, keys);
                       posture.goToPosture("StandInit", 0.5);
                 } catch(const std::exception&){

                     cout << "Error in kick " << endl;
                 }

               std::cout << "Well done!" << std::endl;
               flag = false;
               checking = true; // Empezar a checar si la pelota entro
               posture.goToPosture("StandInit", 0.5);
               //motion.angleInterpolation("HeadPitch", -0.6108, 1, true);
               motion.angleInterpolation("HeadPitch", 0.2722, 1, true); // cabeza en posicion neutral
               camProxy.setActiveCamera(0); // Conectar a camara superior

            //--------------------IMPORTANTE-------------
            // Es la llave del if para las coordenadas originales
            // }
            }

            //Si se cae que se levante
            if ((postura == "LyingBelly")||(postura == "LyingBack")||(postura == "Back")){
                posture.goToPosture("Stand",1);
            }

            //Actualizamos la vista de la camara
            //imshow("Thresholded Image", imgThresholded); //show the thresholded image
            //imshow("Src", frame); //show the original image
            key = waitKey(50);
        }
        contFrames++;
    }
   camProxy.unsubscribe(clientName);
   posture.goToPosture("Crouch", 0.5);

   return 0;
}

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

    /// Show in a window
    namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
    imshow( "Contours", drawing );

    /// Calculate the area with the moments 00 and compare with the result of the OpenCV function
    printf("\t Info: Area and Contour Length \n");
    for( int i = 0; i< contours.size(); i++ ){

         // Se suman los centroides de cada figura y se saca un promedio del centro de los contornos

         //Calculamos el promedio de los centroides
         if(contourArea(contours[i]) >= 150){
         totalContorno++;
         sumX += mc[i].x;
         sumY += mc[i].y;
         }
      }

    // asignamos el promedio y limpiamos las variables
    cout<<"algoritmo nuevo"<<endl;
    avgX = (int) (sumX/totalContorno);
    avgY = (int) (sumY/totalContorno);
    sumX = 0;
    sumY = 0;
    totalContorno = 0;

    /*
    //Verificacmos que algoritmo es el mejor

    cout<<"cuenta contorno = "<<totalContorno<<endl;
    cout<<"total contorno = "<<contours.size()<<endl;

      if(totalContorno > 0 && totalContorno < 4){
          // asignamos el promedio y limpiamos las variables
          cout<<"algoritmo nuevo"<<endl;
          avgX = (int) (sumX/totalContorno);
          avgY = (int) (sumY/totalContorno);
          sumX = 0;
          sumY = 0;
          totalContorno = 0;
      }else{
          cout<<"algoritmo viejo"<<endl;
          Moments oMoments = moments(src_gray);
          double dM01 = oMoments.m01;
          double dM10 = oMoments.m10;
          double dArea = oMoments.m00;

          //calculate the position of the ball
          avgX = dM10 / dArea;
          avgY = dM01 / dArea;
      }

      */

 }
