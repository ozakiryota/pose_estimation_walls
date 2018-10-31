#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

class VelodyneOdometry{
	private:
		ros::NodeHandle nh;
		ros::Subscriber sub;
		ros::Publisher pub;
		nav_msgs::Odometry odom_out;
	public:
		VelodyneOdometry();
		void Callback(const nav_msgs::OdometryConstPtr& msg);
		void BroadcastTF(void);
};

VelodyneOdometry::VelodyneOdometry()
{
	sub = nh.subscribe("/integrated_to_init", 1, &VelodyneOdometry::Callback, this);
	pub = nh.advertise<nav_msgs::Odometry>("/loamvelodyne_odometry", 1);
}

void VelodyneOdometry::Callback(const nav_msgs::OdometryConstPtr& msg)
{
	odom_out = *msg;
	odom_out.header.frame_id = "/odom";
	odom_out.pose.pose.position.x = msg->pose.pose.position.z;
	odom_out.pose.pose.position.y = msg->pose.pose.position.x;
	odom_out.pose.pose.position.z = msg->pose.pose.position.y;
	odom_out.pose.pose.orientation.x = msg->pose.pose.orientation.z;
	odom_out.pose.pose.orientation.y = msg->pose.pose.orientation.x;
	odom_out.pose.pose.orientation.z = msg->pose.pose.orientation.y;
	pub.publish(odom_out);

	BroadcastTF();
}

void VelodyneOdometry::BroadcastTF(void)
{
	static tf::TransformBroadcaster broadcaster;
    geometry_msgs::TransformStamped transform;
	transform.header = odom_out.header;
	transform.child_frame_id = "/loamvelodyne_odometry";
	transform.transform.translation.x = odom_out.pose.pose.position.x;
	transform.transform.translation.y = odom_out.pose.pose.position.y;
	transform.transform.translation.z = odom_out.pose.pose.position.z;
	transform.transform.rotation = odom_out.pose.pose.orientation;
	broadcaster.sendTransform(transform);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "odom_creater_for_loamvelodyne");

	VelodyneOdometry velodyne_odometry;

	ros::spin();
}
