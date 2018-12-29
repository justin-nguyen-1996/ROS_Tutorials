
#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "message_filters/subscriber.h"
#include "tf/message_filter.h"
#include "laser_geometry/laser_geometry.h"
#include "sensor_msgs/PointCloud.h"

class LaserScanToPointCloud{

public:

  ros::NodeHandle n_; // Always need a node handle
  tf::TransformListener listener_; // Need to listen to resulting TF transforms for conversions, etc.
  message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_; // Subscribe to laser scan messages
  tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_; // Filter the laser scan messages until the transform is ready
  laser_geometry::LaserProjection projector_; // Needed to perform the laser scan --> point cloud transform
  ros::Publisher scan_pub_; // Needed to publish the results of the transform

  LaserScanToPointCloud(ros::NodeHandle n) :
    n_(n),
    laser_sub_(n_, "base_scan", 10),
    laser_notifier_(laser_sub_, listener_, "base_link", 10)
  {
//     laser_notifier_.registerCallback( boost::bind(&LaserScanToPointCloud::scanCallback, this, _1) );
    laser_notifier_.registerCallback( boost::bind(&LaserScanToPointCloud::scanCallback, this, _1) );
    laser_notifier_.setTolerance(ros::Duration(0.01));
    scan_pub_ = n_.advertise<sensor_msgs::PointCloud>("/my_cloud", 1);
  }

//   void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
  void scanCallback (const sensor_msgs::LaserScanConstPtr& scan_in)
  {
    sensor_msgs::PointCloud cloud;
    try {
        // Transform the laser scan (scan_in) and put the output in the point cloud object (cloud) in the "base_link"
        // reference frame using the listener_ TF object
        projector_.transformLaserScanToPointCloud("base_link", *scan_in, cloud, listener_);
    }
    catch (tf::TransformException& e) {
        std::cout << e.what();
        return;
    }

    // Do something with cloud.
    scan_pub_.publish(cloud);
  }
};

int main(int argc, char** argv)
{

  ros::init(argc, argv, "my_scan_to_cloud");
  ros::NodeHandle n;
  LaserScanToPointCloud lstopc(n);

  ros::spin();

  return 0;
}
