#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <Eigen/Core>
#include <Eigen/LU>

class CombineLinearVelocityAndYawRate{
	private:
		ros::NodeHandle nh;
		/*subscribe*/
		ros::Subscriber sub_odom;
		ros::Subscriber sub_yawrate;
		/*publish*/
		ros::Publisher pub;
		tf::TransformBroadcaster tf_broadcaster;
		/*odom*/
		nav_msgs::Odometry odom2d_now;
		nav_msgs::Odometry odom2d_last;
		nav_msgs::Odometry odom3d_now;
		nav_msgs::Odometry odom3d_last;
		/*flags*/
		bool first_callback_odom = true;
	public:
		CombineLinearVelocityAndYawRate();
		void InitializeOdom(nav_msgs::Odometry& odom);
		void CallbackOdom(const nav_msgs::OdometryConstPtr& msg);
		void CallbackYawRate(const std_msgs::Float64ConstPtr& msg);
		Eigen::MatrixXd FrameRotation(geometry_msgs::Quaternion q, Eigen::MatrixXd X, bool from_global_to_local);
		void Publisher(void);
};

CombineLinearVelocityAndYawRate::CombineLinearVelocityAndYawRate()
{
	sub_odom = nh.subscribe("/odom", 1, &CombineLinearVelocityAndYawRate::CallbackOdom, this);
	sub_yawrate = nh.subscribe("/yaw_rate_walls", 1, &CombineLinearVelocityAndYawRate::CallbackYawRate, this);
	pub = nh.advertise<nav_msgs::Odometry>("/combined_odometry2d", 1);
	InitializeOdom(odom3d_now);
	InitializeOdom(odom3d_last);
}

void CombineLinearVelocityAndYawRate::InitializeOdom(nav_msgs::Odometry& odom)
{
	odom.header.frame_id = "/odom";
	odom.child_frame_id = "/combined_odometry2d";
	odom.pose.pose.position.x = 0.0;
	odom.pose.pose.position.y = 0.0;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation.x = 0.0;
	odom.pose.pose.orientation.y = 0.0;
	odom.pose.pose.orientation.z = 0.0;
	odom.pose.pose.orientation.w = 1.0;
}

void CombineLinearVelocityAndYawRate::CallbackOdom(const nav_msgs::OdometryConstPtr& msg)
{
	odom2d_now = *msg;
	if(first_callback_odom)	odom2d_last = odom2d_now;
	
	Eigen::MatrixXd GlobalMove2d(3, 1);
	GlobalMove2d <<	odom2d_now.pose.pose.position.x - odom2d_last.pose.pose.position.x,
					odom2d_now.pose.pose.position.y - odom2d_last.pose.pose.position.y,
					odom2d_now.pose.pose.position.z - odom2d_last.pose.pose.position.z;
	Eigen::MatrixXd LocalMove2d = FrameRotation(odom2d_last.pose.pose.orientation, GlobalMove2d, true);
	Eigen::MatrixXd GlobalMove3d = FrameRotation(odom3d_last.pose.pose.orientation, LocalMove2d, false);

	odom3d_now.pose.pose.position.x = odom3d_last.pose.pose.position.x + GlobalMove3d(0, 0);
	odom3d_now.pose.pose.position.y = odom3d_last.pose.pose.position.y + GlobalMove3d(1, 0);
	odom3d_now.pose.pose.position.z = odom3d_last.pose.pose.position.z + GlobalMove3d(2, 0);
	
	odom2d_last = odom2d_now;
	odom3d_last = odom3d_now;
	first_callback_odom = false;

	Publisher();
}

void CombineLinearVelocityAndYawRate::CallbackYawRate(const std_msgs::Float64ConstPtr& msg)
{
	tf::Quaternion q;
	quaternionMsgToTF(odom3d_now.pose.pose.orientation, q);
	double roll, pitch, yaw;
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
	yaw = atan2(sin(yaw + msg->data), cos(yaw + msg->data));
	q = tf::createQuaternionFromRPY(roll, pitch, yaw);
	quaternionTFToMsg(q, odom3d_now.pose.pose.orientation);
}

Eigen::MatrixXd CombineLinearVelocityAndYawRate::FrameRotation(geometry_msgs::Quaternion q, Eigen::MatrixXd X, bool from_global_to_local)
{
	Eigen::MatrixXd Rot(3, 3); 
	Rot <<  q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z,  2*(q.x*q.y + q.w*q.z),  2*(q.x*q.z - q.w*q.y),
			2*(q.x*q.y - q.w*q.z),  q.w*q.w - q.x*q.x + q.y*q.y - q.z*q.z,  2*(q.y*q.z + q.w*q.x),
			2*(q.x*q.z + q.w*q.y),  2*(q.y*q.z - q.w*q.x),  q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z;
	// std::cout << "X = " << std::endl << X << std::endl;
	if(from_global_to_local)    return Rot*X;
	else    return Rot.inverse()*X;
}

void CombineLinearVelocityAndYawRate::Publisher(void)
{
	/*publish*/
	pub.publish(odom3d_now);
	/*tf broadcast */
    geometry_msgs::TransformStamped transform;
	transform.header.stamp = ros::Time::now();
	transform.header.frame_id = "/odom";
	transform.child_frame_id = "/combined_odometry2d";
	transform.transform.translation.x = odom3d_now.pose.pose.position.x;
	transform.transform.translation.y = odom3d_now.pose.pose.position.y;
	transform.transform.translation.z = odom3d_now.pose.pose.position.z;
	transform.transform.rotation = odom3d_now.pose.pose.orientation;
	tf_broadcaster.sendTransform(transform);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "combine_linearvelocity_and_yawrate");

	CombineLinearVelocityAndYawRate combine_linear_velocity_and_yawrate;

	ros::spin();
}
