#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgcodecs/imgcodecs.hpp"
#include <stdlib.h>
#include <stdio.h>

using namespace cv;
using namespace std;

/** @function main */
int main( int argc, char** argv )
{

  Mat src, src_gray, orig;
  Mat grad;
  char* window_name = "Sobel Demo - Simple Edge Detector";
  int scale = 1;
  int delta = 0;
  int ddepth = CV_16S;

  int c;
  VideoCapture cam;
  
  if(!cam.open(0))
    return 0;
  

  /// Load an image
  //src = imread( argv[1] );
  while(true)
  {
    cam >> src;
    orig = src;
    if( !src.data )
    { 
      return -1; 
    }

    GaussianBlur( src, src, Size(3,3), 0, 0, BORDER_DEFAULT );

    /// Convert it to gray
    cvtColor( src, src_gray, CV_BGR2GRAY );

    /// Create window
    namedWindow( window_name, CV_WINDOW_AUTOSIZE );

    /// Generate grad_x and grad_y
    Mat grad_x, grad_y;
    Mat abs_grad_x, abs_grad_y;

    /// Gradient X
    //Scharr( src_gray, grad_x, ddepth, 1, 0, scale, delta, BORDER_DEFAULT );
    Sobel( src_gray, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT );
    convertScaleAbs( grad_x, abs_grad_x );

    /// Gradient Y
    //Scharr( src_gray, grad_y, ddepth, 0, 1, scale, delta, BORDER_DEFAULT );
    Sobel( src_gray, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT );
    convertScaleAbs( grad_y, abs_grad_y );

    /// Total Gradient (approximate)
    addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad );

    vector<Vec3f> circles;
    HoughCircles(grad, circles, HOUGH_GRADIENT, 1,
                 grad.rows/16, // change this value to detect circles with different distances to each other
                 100, 30, 5, 50 // change the last two parameters
                                // (min_radius & max_radius) to detect larger circles
                 );
    for( size_t i = 0; i < circles.size(); i++ )
    {
        Vec3i c = circles[i];
        circle( orig, Point(c[0], c[1]), c[2], Scalar(0,0,255), 3, LINE_AA);
        circle( orig, Point(c[0], c[1]), 2, Scalar(0,255,0), 3, LINE_AA);
    }


    imshow( window_name, grad );
    imshow("Original", orig);

    if(waitKey(30) >= 0)
      break;
  }

  return 0;
  }
