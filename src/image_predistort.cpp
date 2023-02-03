#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image window";

class ImagePredistort
{
  ros::NodeHandle nh_;

  // image transport
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImagePredistort()
    : it_(nh_)
  {
    // subscribe to
    image_sub_ = it_.subscribeCamera("image", 1, &ImagePredistort::imageCb, this);
    image_pub_ = it_.advertise("image_predistort", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImagePredistort()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& info)
  {
    // cv_bridge boilerplate
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // convert CameraInfo message to cv::Mat
    cv::Mat intr_k(3, 3, CV_64FC1, (void *)info->K.data());
    cv::Mat intr_d(1, 5, CV_64FC1, (void *)info->D.data());

    // referenced https://github.com/mightbxg/DistortImage/blob/master/distort.cpp
    // I have no idea how this works or even if it works
    cv::Mat map_x = cv::Mat(cv_ptr->image.size(), CV_32FC1);
    cv::Mat map_y = cv::Mat(cv_ptr->image.size(), CV_32FC1);

    cv::Mat R;

    std::vector<cv::Point2f> pts_ud, pts_dist;

    for (int y = 0; y < cv_ptr->image.size().height; ++y) {
      for (int x = 0; x < cv_ptr->image.size().width; ++x) {
        pts_dist.emplace_back(x, y);
      }
    }

    cv::undistortPoints(pts_dist, pts_ud, intr_k, intr_d, R, intr_k);
    for (int y = 0; y < cv_ptr->image.size().height; ++y) {
      float* ptr1 = map_x.ptr<float>(y);
      float* ptr2 = map_y.ptr<float>(y);
      for (int x = 0; x < cv_ptr->image.size().width; ++x) {
        const auto& pt = pts_ud[y * cv_ptr->image.size().width + x];
        ptr1[x] = pt.x;
        ptr2[x] = pt.y;
      }
    }

    cv::Mat img_dist;
    cv::remap(cv_ptr->image, img_dist, map_x, map_y, cv::INTER_LINEAR);


    // undistort
    // cv::Mat map_x, map_y;
    // cv::initUndistortRectifyMap(intr_k, intr_d, cv::Mat(), intr_k, cv_ptr->image.size(), CV_32FC1, map_x, map_y);
    // cv::Mat img_dist;
    // cv::remap(cv_ptr->image, img_dist, map_x, map_y, cv::INTER_LINEAR);

    // show debug view
    // cv::imshow(OPENCV_WINDOW, img_dist);
    // cv::waitKey(3);

    // publish predistorted image
    cv_ptr->image = img_dist;
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_predistort");
  ImagePredistort image_predistort;
  ros::spin();
  return 0;
}
