
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>

int main(int argc, char** argv){

  // But really the node will be renamed to listener in the .launch file
  ros::init(argc, argv, "my_tf_listener");

  ros::NodeHandle node;

  // Spawn a new turtle in the same world (window) as the first turtle
  // TODO:
  ros::service::waitForService("spawn");
  ros::ServiceClient add_turtle =
    node.serviceClient<turtlesim::Spawn>("spawn");
  turtlesim::Spawn srv;
  add_turtle.call(srv);

  // This listener node publishes velocities to the turtle2/cmd_vel topic
  ros::Publisher turtle_vel =
    node.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 10);

  tf::TransformListener listener;

  ros::Rate rate(10.0);
  while (node.ok()){
    tf::StampedTransform transform;
    try{
      // Find the transform that gives turtle1's data in turtle2's frame of reference.
      // Find the latest transform and store it in the `transform` object.
      ros::Time now = ros::Time::now();
      listener.waitForTransform("/turtle2", "/turtle1", now, ros::Duration(3.0));
      listener.lookupTransform("/turtle2", "/turtle1", now, transform); // Turtle 2 follows turtle 1.
//       listener.lookupTransform("/turtle2", "carrot1", ros::Time(0), transform); // Turtle 2 follows carrot1
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    ROS_INFO("transform.getOrigin().y(): %f\n", transform.getOrigin().y());
    ROS_INFO("transform.getOrigin().x(): %f\n", transform.getOrigin().x());

    // Change the angular z-speed and linear x-speed based on the transform needed to go from turtle2's pose and
    // velocities to turtle1's pose and velocities.
    geometry_msgs::Twist vel_msg;
    vel_msg.angular.z = 4.0 * atan2(transform.getOrigin().y(),
                                    transform.getOrigin().x());
    vel_msg.linear.x = 0.5 * sqrt(pow(transform.getOrigin().x(), 2) +
                                  pow(transform.getOrigin().y(), 2));
//     vel_msg.linear.x = 1.5;

    // Then send those to turtle2/cmd_vel.
    turtle_vel.publish(vel_msg);

    rate.sleep();
  }
  return 0;
};
