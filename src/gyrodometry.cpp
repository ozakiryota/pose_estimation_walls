#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Core>
#include <Eigen/LU>
#include <sensor_msgs/Imu.h>

class Gyrodometry{
	private:
		ros::NodeHandle nh;
		/*subscribe*/
		ros::Subscriber sub_inipose;
		ros::Subscriber sub_odom;
		ros::Subscriber sub_imu;
		ros::Subscriber sub_bias;
		/*publish*/
		ros::Publisher pub;
		/*odom*/
		nav_msgs::Odometry odom2d_now;
		nav_msgs::Odometry odom2d_last;
		nav_msgs::Odometry odom3d_now;
		nav_msgs::Odometry odom3d_last;
		/*objects*/
		tf::Quaternion q_pose;
		sensor_msgs::Imu bias;
		sensor_msgs::Imu imu_last;
		/*time*/
		ros::Time time_now_imu;
		ros::Time time_last_imu;
		/*flags*/
		bool first_callback_odom = true;
		bool first_callback_imu = true;
		bool inipose_is_available = false;
		bool bias_is_available = false;
	public:
		Gyrodometry();
		void InitializeOdom(nav_msgs::Odometry& odom);
		void CallbackInipose(const geometry_msgs::QuaternionConstPtr& msg);
		void CallbackOdom(const nav_msgs::OdometryConstPtr& msg);
		void CallbackIMU(const sensor_msgs::ImuConstPtr& msg);
		void CallbackBias(const sensor_msgs::ImuConstPtr& msg);
		Eigen::MatrixXd FrameRotation(geometry_msgs::Quaternion q, Eigen::MatrixXd X, bool from_global_to_local);
		void Publisher(void);
};

Gyrodometry::Gyrodometry()
{
	sub_inipose = nh.subscribe("/initial_pose", 1, &Gyrodometry::CallbackInipose, this);
	sub_odom = nh.subscribe("/odom", 1, &Gyrodometry::CallbackOdom, this);
	sub_imu = nh.subscribe("/imu/data", 1, &Gyrodometry::CallbackIMU, this);
	sub_bias = nh.subscribe("/imu_bias", 1, &Gyrodometry::CallbackBias, this);
	pub = nh.advertise<nav_msgs::Odometry>("/gyrodometry", 1);
	InitializeOdom(odom3d_now);
	InitializeOdom(odom3d_last);
}

void Gyrodometry::InitializeOdom(nav_msgs::Odometry& odom)
{
	odom.header.frame_id = "/odom";
	odom.child_frame_id = "/gyrodometry";
	odom.pose.pose.position.x = 0.0;
	odom.pose.pose.position.y = 0.0;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation.x = 0.0;
	odom.pose.pose.orientation.y = 0.0;
	odom.pose.pose.orientation.z = 0.0;
	odom.pose.pose.orientation.w = 1.0;
}

void Gyrodometry::CallbackInipose(const geometry_msgs::QuaternionConstPtr& msg)
{
	if(!inipose_is_available){
		quaternionMsgToTF(*msg, q_pose);
		inipose_is_available = true;
	}   
}

void Gyrodometry::CallbackOdom(const nav_msgs::OdometryConstPtr& msg)
{
	odom2d_now = *msg;
	if(first_callback_odom)	odom2d_last = odom2d_now;
	
	Eigen::MatrixXd GlobalMove2d(3, 1);
	GlobalMove2d <<	odom2d_now.pose.pose.position.x - odom2d_last.pose.pose.position.x,
					odom2d_now.pose.pose.position.y - odom2d_last.pose.pose.position.y,
					odom2d_now.pose.pose.position.z - odom2d_last.pose.pose.position.z;
	// Eigen::MatrixXd LocalMove2d = FrameRotation(odom2d_last.pose.pose.orientation, GlobalMove2d, true);
	// Eigen::MatrixXd GlobalMove3d = FrameRotation(odom3d_last.pose.pose.orientation, LocalMove2d, false);
	Eigen::MatrixXd LocalMove2d = FrameRotation(odom2d_now.pose.pose.orientation, GlobalMove2d, true);
	Eigen::MatrixXd GlobalMove3d = FrameRotation(odom3d_now.pose.pose.orientation, LocalMove2d, false);

	odom3d_now.pose.pose.position.x = odom3d_last.pose.pose.position.x + GlobalMove3d(0, 0);
	odom3d_now.pose.pose.position.y = odom3d_last.pose.pose.position.y + GlobalMove3d(1, 0);
	odom3d_now.pose.pose.position.z = odom3d_last.pose.pose.position.z + GlobalMove3d(2, 0);
	
	odom2d_last = odom2d_now;
	odom3d_last = odom3d_now;
	first_callback_odom = false;

	Publisher();
}
void Gyrodometry::CallbackIMU(const sensor_msgs::ImuConstPtr& msg)
{
	time_now_imu = ros::Time::now();
	double dt;
	try{
		dt = (time_now_imu - time_last_imu).toSec();
	}
	catch(std::runtime_error& ex) {
		ROS_ERROR("Exception: [%s]", ex.what());
	}
	time_last_imu = time_now_imu;
	if(first_callback_imu){
		dt = 0.0;
		imu_last = *msg;
	}
	else if(inipose_is_available){
		double delta_r = (msg->angular_velocity.x + imu_last.angular_velocity.x)*dt/2.0;
		double delta_p = (msg->angular_velocity.y + imu_last.angular_velocity.y)*dt/2.0;
		double delta_y = (msg->angular_velocity.z + imu_last.angular_velocity.z)*dt/2.0;
		if(bias_is_available){
			delta_r -= bias.angular_velocity.x*dt;
			delta_p -= bias.angular_velocity.y*dt;
			delta_y -= bias.angular_velocity.z*dt;
		}
		tf::Quaternion q_relative_rotation = tf::createQuaternionFromRPY(delta_r, delta_p, delta_y);
		q_pose = q_pose*q_relative_rotation;
		q_pose.normalize();
		quaternionTFToMsg(q_pose, odom3d_now.pose.pose.orientation);
	}

	imu_last = *msg;
	first_callback_imu = false;
}
void Gyrodometry::CallbackBias(const sensor_msgs::ImuConstPtr& msg)
{
	if(!bias_is_available){
		bias = *msg;
		bias_is_available = true;
	}
}

Eigen::MatrixXd Gyrodometry::FrameRotation(geometry_msgs::Quaternion q, Eigen::MatrixXd X, bool from_global_to_local)
{
	Eigen::MatrixXd Rot(3, 3); 
	Rot <<  q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z,  2*(q.x*q.y + q.w*q.z),  2*(q.x*q.z - q.w*q.y),
			2*(q.x*q.y - q.w*q.z),  q.w*q.w - q.x*q.x + q.y*q.y - q.z*q.z,  2*(q.y*q.z + q.w*q.x),
			2*(q.x*q.z + q.w*q.y),  2*(q.y*q.z - q.w*q.x),  q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z;
	// std::cout << "X = " << std::endl << X << std::endl;
	if(from_global_to_local)    return Rot*X;
	else    return Rot.inverse()*X;
}

void Gyrodometry::Publisher(void)
{
	/*publish*/
	pub.publish(odom3d_now);
	/*tf broadcast*/
	static tf::TransformBroadcaster broadcaster;
    geometry_msgs::TransformStamped transform;
	transform.header.stamp = ros::Time::now();
	transform.header.frame_id = "/odom";
	transform.child_frame_id = "/gyrodometry";
	transform.transform.translation.x = odom3d_now.pose.pose.position.x;
	transform.transform.translation.y = odom3d_now.pose.pose.position.y;
	transform.transform.translation.z = odom3d_now.pose.pose.position.z;
	transform.transform.rotation = odom3d_now.pose.pose.orientation;
	broadcaster.sendTransform(transform);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "gyrodometry");

	Gyrodometry gyrodometry;

	ros::spin();
}
