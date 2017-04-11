//Includes all the headers necessary to use the most common public pieces of the ROS system.
#include <ros/ros.h>
//Use image_transport for publishing and subscribing to images in ROS
#include <image_transport/image_transport.h>
//Use cv_bridge to convert between ROS and OpenCV Image formats
#include <cv_bridge/cv_bridge.h>
//Include some useful constants for image encoding. Refer to: http://www.ros.org/doc/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html for more info.
#include <sensor_msgs/image_encodings.h>

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/xfeatures2d.hpp"

#include <stdio.h>
#include <iostream>

//Declare a string with the name of the window that we will create using OpenCV where processed images will be displayed.
static const char WINDOW[] = "Keypoints Sift";

//Use method of ImageTransport to create image publisher
image_transport::Publisher pub;

//This function is called everytime a new image is published
void imageCallback(const sensor_msgs::ImageConstPtr& original_image)
{
  //Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    //Always copy, returning a mutable CvImage
    //OpenCV expects color images to use BGR channel order.
    cv_ptr = cv_bridge::toCvCopy(original_image, sensor_msgs::image_encodings::MONO8);
  } catch (cv_bridge::Exception& e)
  {
    //if there is an error during conversion, display it
    ROS_ERROR("cam_sift_cpp_node.cpp cv_bridge exception: %s", e.what());
    return;
  }

  //-- Step 1: Detect the keypoints using SIFT Detector
  cv::Ptr<cv::xfeatures2d::SIFT> detector = cv::xfeatures2d::SIFT::create();
  std::vector<cv::KeyPoint> keypoints;
  detector->detect( cv_ptr->image, keypoints );

  //-- Draw keypoints
  cv::Mat img_keypoints;
  cv::drawKeypoints( cv_ptr->image,  keypoints, img_keypoints, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

  //-- Show detected (drawn) keypoints
  cv::imshow(WINDOW, img_keypoints);

  //Add some delay in miliseconds. The function only works if there is at least one HighGUI window created and the window is active. If there are several HighGUI windows, any of them can be active.
  cv::waitKey(3);

  /**
   * The publish() function is how you send messages. The parameter
   * is the message object. The type of this object must agree with the type
   * given as a template parameter to the advertise<>() call, as was done
   * in the constructor in main().
   */
  //Convert the CvImage to a ROS image message and publish it on the "image_topic_out" topic.
  // convert a cv::Mat in cv_bridge::CvImagePtr
  cv_bridge::CvImagePtr cv_ptr_keypoints(new cv_bridge::CvImage());
  cv_ptr_keypoints->image = img_keypoints;
  cv_ptr_keypoints->header = cv_ptr->header;
  cv_ptr_keypoints->encoding = sensor_msgs::image_encodings::BGR8;

  pub.publish(cv_ptr_keypoints->toImageMsg());
}

/**
 * This tutorial demonstrates simple image conversion between ROS image message and OpenCV formats and image processing
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node. Node names must be unique in a running system.
   * The name used here must be a base name, ie. it cannot have a / in it.
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "cam_sift_cpp_node");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle nh;

  //Create an ImageTransport instance, initializing it with our NodeHandle.
  image_transport::ImageTransport it(nh);

  //OpenCV HighGUI call to create a display window on start-up.
  cv::namedWindow(WINDOW, CV_WINDOW_AUTOSIZE);

  /**
   * Subscribe to the "image_topic_in" base topic. The actual ROS topic subscribed to depends on which transport is used.
   * ROS will call the "imageCallback" function whenever a new image arrives. The 2nd argument is the queue size.
   * subscribe() returns an image_transport::Subscriber object, that you must hold on to until you want to unsubscribe.
   * When the Subscriber object is destructed, it will automatically unsubscribe from the "image_topic_in" base topic.
   */
  image_transport::Subscriber sub = it.subscribe("image_topic_in", 1, imageCallback);

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  pub = it.advertise("image_topic_out", 1);

  /**
   * In this application all user callbacks will be called from within the ros::spin() call.
   * ros::spin() will not return until the node has been shutdown, either through a call
   * to ros::shutdown() or a Ctrl-C.
   */
  ros::spin();

  //OpenCV HighGUI call to destroy a display window on shut-down.
  cv::destroyWindow(WINDOW);
}
