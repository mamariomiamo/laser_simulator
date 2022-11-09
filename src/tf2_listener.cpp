#include <ros/ros.h>
// #include <string>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
geometry_msgs::TransformStamped transformStamped;
sensor_msgs::PointCloud2 local_cloud_world;
ros::Publisher local_cloud_world_pub;

void cloudcallback(const sensor_msgs::PointCloud2 &local_cloud)
{
  // std::cout << "in call back " << std::endl;
  tf2::doTransform(local_cloud, local_cloud_world, transformStamped);
  local_cloud_world_pub.publish(local_cloud_world);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "my_tf2_listener");

  ros::NodeHandle node("~"); // ~ is important so the namespace is added in front of the ros parameters

  double publish_rate;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  std::string cloud_in_topic, cloud_out_topic;

  // node.param<std::string>("cloud_in_topic", cloud_in_topic, "cloud_in");
  // node.param<std::string>("cloud_out_topic", cloud_out_topic, "cloud_out");
  // node.param<double>("publish_rate", publish_rate, 10);

  node.getParam("cloud_in_topic", cloud_in_topic);
  node.getParam("cloud_out_topic", cloud_out_topic);
  node.getParam("publish_rate", publish_rate);

  std::cout << "cloud_in_topic is: " << cloud_in_topic << std::endl;
  std::cout << "cloud_out_topic is: " << cloud_out_topic << std::endl;
  std::cout << "publish_rate is: " << publish_rate << std::endl;

  ros::Subscriber local_cloud = node.subscribe(cloud_in_topic, 50, cloudcallback);
  local_cloud_world_pub = node.advertise<sensor_msgs::PointCloud2>(cloud_out_topic, 10);

  ros::Rate rate(publish_rate);
  while (node.ok())
  {
    // std::cout << "publish_rate is " << publish_rate << std::endl;

    try
    {
      transformStamped = tfBuffer.lookupTransform("world", "laser",
                                                  ros::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("%s", ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
};