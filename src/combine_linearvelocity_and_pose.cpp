#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Core>
#include <Eigen/LU>

class CombineLinearVelocityAndPose{
	private:
		ros::NodeHandle nh;
		/*subscribe*/
		ros::Subscriber sub_odom;
		ros::Subscriber sub_pose;
		/*publish*/
		ros::Publisher pub;
		/*time*/
		ros::Time time_odom_now;
		ros::Time time_odom_last;
		/*objects*/
		nav_msgs::Odometry odom_now;
		nav_msgs::Odometry odom_last;
		Eigen::MatrixXd Position;
		/*flags*/
		bool first_callback_odom = true;
	public:
		CombineLinearVelocityAndPose();
		void InitializeOdom(nav_msgs::Odometry& odom);
		void CallbackOdom(const nav_msgs::OdometryConstPtr& msg);
		void CallbackPose(const geometry_msgs::PoseStampedConstPtr& msg);
		Eigen::MatrixXd FrameRotation(geometry_msgs::Quaternion q, Eigen::MatrixXd X, bool from_global_to_local);
		void Publisher(void);
};

CombineLinearVelocityAndPose::CombineLinearVelocityAndPose()
{
	sub_odom = nh.subscribe("/odom", 1, &CombineLinearVelocityAndPose::CallbackOdom, this);
	sub_pose = nh.subscribe("/pose_ekf", 1, &CombineLinearVelocityAndPose::CallbackPose, this);
	pub = nh.advertise<nav_msgs::Odometry>("/combined_odometry", 1);
	Position = Eigen::MatrixXd::Constant(3, 1, 0.0);
	InitializeOdom(odom_now);
	InitializeOdom(odom_last);
}

void CombineLinearVelocityAndPose::InitializeOdom(nav_msgs::Odometry& odom)
{
	odom.header.frame_id = "/odom";
	odom.child_frame_id = "/combined_odometry";
	odom.pose.pose.position.x = 0.0;
	odom.pose.pose.position.y = 0.0;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation.x = 0.0;
	odom.pose.pose.orientation.y = 0.0;
	odom.pose.pose.orientation.z = 0.0;
	odom.pose.pose.orientation.w = 1.0;
}

void CombineLinearVelocityAndPose::CallbackOdom(const nav_msgs::OdometryConstPtr& msg)
{
	time_odom_now = ros::Time::now();
	double dt = (time_odom_now - time_odom_last).toSec();
	time_odom_last = time_odom_now;

	odom_now.twist = msg->twist;

	if(first_callback_odom){
		dt = 0.0;
		odom_last = odom_now;
	}
	
	Eigen::MatrixXd LocalVel(3, 1);
	LocalVel <<	msg->twist.twist.linear.x,
		  		msg->twist.twist.linear.y,
				msg->twist.twist.linear.z;
	Eigen::MatrixXd GlobalVel = FrameRotation(odom_last.pose.pose.orientation, LocalVel, false);
	Position = Position + GlobalVel*dt;

	odom_now.pose.pose.position.x = Position(0, 0);
	odom_now.pose.pose.position.y = Position(1, 0);
	odom_now.pose.pose.position.z = Position(2, 0);

	odom_last = odom_now;
	
	Publisher();

	first_callback_odom = false;
}

void CombineLinearVelocityAndPose::CallbackPose(const geometry_msgs::PoseStampedConstPtr& msg)
{
	odom_now.pose.pose.orientation = msg->pose.orientation;
}

Eigen::MatrixXd CombineLinearVelocityAndPose::FrameRotation(geometry_msgs::Quaternion q, Eigen::MatrixXd X, bool from_global_to_local)
{
	Eigen::MatrixXd Rot(3, 3); 
	Rot <<  q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z,  2*(q.x*q.y + q.w*q.z),  2*(q.x*q.z - q.w*q.y),
			2*(q.x*q.y - q.w*q.z),  q.w*q.w - q.x*q.x + q.y*q.y - q.z*q.z,  2*(q.y*q.z + q.w*q.x),
			2*(q.x*q.z + q.w*q.y),  2*(q.y*q.z - q.w*q.x),  q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z;
	// std::cout << "X = " << std::endl << X << std::endl;
	if(from_global_to_local)    return Rot*X;
	else    return Rot.inverse()*X;
}

void CombineLinearVelocityAndPose::Publisher(void)
{
	/*publish*/
	pub.publish(odom_now);
	/*tf*/
	static tf::TransformBroadcaster broadcaster;
    geometry_msgs::TransformStamped transform;
	transform.header.stamp = ros::Time::now();
	transform.header.frame_id = "/odom";
	transform.child_frame_id = "/combined_odometry";
	transform.transform.translation.x = odom_now.pose.pose.position.x;
	transform.transform.translation.y = odom_now.pose.pose.position.y;
	transform.transform.translation.z = odom_now.pose.pose.position.z;
	transform.transform.rotation = odom_now.pose.pose.orientation;
	broadcaster.sendTransform(transform);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "combine_linearvelocity_and_pose");

	CombineLinearVelocityAndPose combine_linear_velocity_and_pose;

	ros::spin();
}
