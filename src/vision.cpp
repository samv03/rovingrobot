#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <queue>
#include <iostream>

using namespace std;
using namespace cv;
queue <cv::Mat> incoming_queue;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    //cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    Mat tempmat = cv_bridge::toCvShare(msg, "bgr8")->image;
    incoming_queue.push(tempmat);
    //imshow("view", incoming_queue.front());
    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  cv::namedWindow("view");
  //cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback);

  Mat src, src_gray, orig;
  Mat grad;
  char* window_name = "Sobel Demo - Simple Edge Detector";
  cv::namedWindow(window_name);
  cv::namedWindow("Original");
  int scale = 1;
  int delta = 0;
  int ddepth = CV_16S;

  int c;

  while(true)
  {
    cout << "Size: " << incoming_queue.size() << endl;
    if(!incoming_queue.empty())
    {
      cv::Mat singleMat;
      singleMat = incoming_queue.front();
      
      imshow("test", singleMat);
      src = singleMat;
      orig = singleMat;
      
      cout << "Test" << endl;
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

      incoming_queue.pop();

      if(waitKey(30) >= 0)
        break;
    }
    ros::spinOnce();
  }
  cv::destroyWindow("view");
}
