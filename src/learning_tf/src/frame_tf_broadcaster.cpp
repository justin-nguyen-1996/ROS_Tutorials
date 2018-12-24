
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_broadcaster");
  ros::NodeHandle node;

  tf::TransformBroadcaster br;
  tf::Transform transform;

  // NOTE: The carrot is placed relative to Turtle1's reference frame. The positive x-direction is always forward
  // relative to the object and the positive y-direction is always left relative to the object. I.e. (0, 2, 0) is not
  // "west" because that would be relative to the world. Instead, it's actually left of where the robot is facing.

  ros::Rate rate(10.0);
  while (node.ok()){
//     transform.setOrigin( tf::Vector3(0.0, 2.0, 0.0) ); // Carrot1 is 2 meters left of Turtle1
//     transform.setOrigin( tf::Vector3(-2.0, 0.0, 0.0) ); // Carrot1 is 2 meters behind Turtle1
    transform.setOrigin(tf::Vector3(2.0*sin(ros::Time::now().toSec()), 2.0*cos(ros::Time::now().toSec()), 0.0)); // Moving carrot changes with time (carrot moves in a circle around turtle1 with distance from turtle1 ranging from [-2,2])
    transform.setRotation(tf::Quaternion(0, 0, 0, 1));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "turtle1", "carrot1"));
    rate.sleep();
  }
  return 0;
};
