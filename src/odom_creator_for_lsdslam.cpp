#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Core>
#include <Eigen/LU>

class OdomCreatorForLSDSLAM{
	private:
		ros::NodeHandle nh;
		/*subscribe*/
		ros::Subscriber sub_inipose;
		ros::Subscriber sub_odom;
		ros::Subscriber sub_pose;
		/*publish*/
		ros::Publisher pub;
		tf::TransformBroadcaster tf_broadcaster;
		/*odom*/
		nav_msgs::Odometry odom2d_now;
		nav_msgs::Odometry odom2d_last;
		nav_msgs::Odometry odom3d_now;
		nav_msgs::Odometry odom3d_last;
		/*objects*/
		tf::Quaternion q_pose = {0.0, 0.0, 0.0, 1.0};
		tf::Quaternion q_slam_now;
		tf::Quaternion q_slam_last;
		/*flags*/
		bool first_callback_odom = true;
		bool inipose_is_available = false;
	public:
		OdomCreatorForLSDSLAM();
		void InitializeOdom(nav_msgs::Odometry& odom);
		void CallbackIniPose(const geometry_msgs::QuaternionConstPtr& msg);
		void CallbackOdom(const nav_msgs::OdometryConstPtr& msg);
		void CallbackPose(const geometry_msgs::PoseStampedConstPtr& msg);
		void Publisher(void);
};

OdomCreatorForLSDSLAM::OdomCreatorForLSDSLAM()
{
	sub_inipose = nh.subscribe("/initial_pose", 1, &OdomCreatorForLSDSLAM::CallbackIniPose, this);
	sub_odom = nh.subscribe("/odom", 1, &OdomCreatorForLSDSLAM::CallbackOdom, this);
	sub_pose = nh.subscribe("/lsd_slam/pose", 1, &OdomCreatorForLSDSLAM::CallbackPose, this);
	pub = nh.advertise<nav_msgs::Odometry>("/lsdslam_odometry", 1);
	InitializeOdom(odom3d_now);
	InitializeOdom(odom3d_last);
}

void OdomCreatorForLSDSLAM::InitializeOdom(nav_msgs::Odometry& odom)
{
	odom.header.frame_id = "/odom";
	odom.child_frame_id = "/lsdslam_odometry";
	odom.pose.pose.position.x = 0.0;
	odom.pose.pose.position.y = 0.0;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation.x = 0.0;
	odom.pose.pose.orientation.y = 0.0;
	odom.pose.pose.orientation.z = 0.0;
	odom.pose.pose.orientation.w = 1.0;
}

void OdomCreatorForLSDSLAM::CallbackIniPose(const geometry_msgs::QuaternionConstPtr& msg)
{
	if(!inipose_is_available){
		quaternionMsgToTF(*msg, q_pose);
		inipose_is_available = true;
	}   
}

void OdomCreatorForLSDSLAM::CallbackOdom(const nav_msgs::OdometryConstPtr& msg)
{
	odom2d_now = *msg;
	if(!first_callback_odom){	
		tf::Quaternion q_pose_odom2d_last;
		tf::Quaternion q_pose_odom3d_last;
		quaternionMsgToTF(odom2d_last.pose.pose.orientation, q_pose_odom2d_last);
		quaternionMsgToTF(odom3d_last.pose.pose.orientation, q_pose_odom3d_last);
		tf::Quaternion q_global_move2d = tf::Quaternion(
				odom2d_now.pose.pose.position.x - odom2d_last.pose.pose.position.x,
				odom2d_now.pose.pose.position.y - odom2d_last.pose.pose.position.y,
				odom2d_now.pose.pose.position.z - odom2d_last.pose.pose.position.z,
				0.0);
		tf::Quaternion q_local_move2d = q_pose_odom2d_last.inverse()*q_global_move2d*q_pose_odom2d_last;
		tf::Quaternion q_global_move3d = q_pose_odom3d_last*q_local_move2d*q_pose_odom3d_last.inverse();

		odom3d_now.pose.pose.position.x = odom3d_last.pose.pose.position.x + q_global_move3d.x();
		odom3d_now.pose.pose.position.y = odom3d_last.pose.pose.position.y + q_global_move3d.y();
		odom3d_now.pose.pose.position.z = odom3d_last.pose.pose.position.z + q_global_move3d.z();
	}
	
	odom2d_last = odom2d_now;
	odom3d_last = odom3d_now;
	first_callback_odom = false;

	Publisher();
}

void OdomCreatorForLSDSLAM::CallbackPose(const geometry_msgs::PoseStampedConstPtr& msg)
{
	q_slam_now = tf::Quaternion(msg->pose.orientation.z, -msg->pose.orientation.x, -msg->pose.orientation.y, msg->pose.orientation.w);
	q_slam_now.normalize();
	
	if(inipose_is_available){
		tf::Quaternion q_relative_rotation = q_slam_last.inverse()*q_slam_now;
		q_relative_rotation.normalize();
		q_pose = q_pose*q_relative_rotation;
		q_pose.normalize();
		quaternionTFToMsg(q_pose, odom3d_now.pose.pose.orientation);
	}
	
	q_slam_last = q_slam_now;
}

void OdomCreatorForLSDSLAM::Publisher(void)
{
	/*publish*/
	pub.publish(odom3d_now);
	/*tf broadcast */
    geometry_msgs::TransformStamped transform;
	transform.header.stamp = ros::Time::now();
	transform.header.frame_id = "/odom";
	transform.child_frame_id = "/lsdslam_odometry";
	transform.transform.translation.x = odom3d_now.pose.pose.position.x;
	transform.transform.translation.y = odom3d_now.pose.pose.position.y;
	transform.transform.translation.z = odom3d_now.pose.pose.position.z;
	transform.transform.rotation = odom3d_now.pose.pose.orientation;
	tf_broadcaster.sendTransform(transform);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "odom_creater_for_lsdslam");

	OdomCreatorForLSDSLAM odom_creater_for_lsdslam;

	ros::spin();
}
