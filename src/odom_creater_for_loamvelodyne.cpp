#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

class OdomCreaterForLoamVelodyne{
	private:
		ros::NodeHandle nh;
		ros::Subscriber sub;
		ros::Publisher pub;
		nav_msgs::Odometry odom_out;
		tf::TransformBroadcaster tf_broadcaster;
	public:
		OdomCreaterForLoamVelodyne();
		void Callback(const nav_msgs::OdometryConstPtr& msg);
		void BroadcastTF(void);
};

OdomCreaterForLoamVelodyne::OdomCreaterForLoamVelodyne()
{
	sub = nh.subscribe("/integrated_to_init", 1, &OdomCreaterForLoamVelodyne::Callback, this);
	pub = nh.advertise<nav_msgs::Odometry>("/loamvelodyne_odometry", 1);
}

void OdomCreaterForLoamVelodyne::Callback(const nav_msgs::OdometryConstPtr& msg)
{
	odom_out = *msg;
	odom_out.header.frame_id = "/odom";
	odom_out.child_frame_id = "/loamvelodyne_odometry";
	odom_out.pose.pose.position.x = msg->pose.pose.position.z;
	odom_out.pose.pose.position.y = msg->pose.pose.position.x;
	odom_out.pose.pose.position.z = msg->pose.pose.position.y;
	odom_out.pose.pose.orientation.x = msg->pose.pose.orientation.z;
	odom_out.pose.pose.orientation.y = msg->pose.pose.orientation.x;
	odom_out.pose.pose.orientation.z = msg->pose.pose.orientation.y;
	pub.publish(odom_out);

	BroadcastTF();
}

void OdomCreaterForLoamVelodyne::BroadcastTF(void)
{
    geometry_msgs::TransformStamped transform;
	transform.header = odom_out.header;
	transform.child_frame_id = "/loamvelodyne_odometry";
	transform.transform.translation.x = odom_out.pose.pose.position.x;
	transform.transform.translation.y = odom_out.pose.pose.position.y;
	transform.transform.translation.z = odom_out.pose.pose.position.z;
	transform.transform.rotation = odom_out.pose.pose.orientation;
	tf_broadcaster.sendTransform(transform);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "odom_creater_for_loamvelodyne");

	OdomCreaterForLoamVelodyne velodyne_odometry;

	ros::spin();
}
