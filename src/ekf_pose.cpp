#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/tf.h>
// #include <Eigen/Core>
// #include <Eigen/LU>

class EKFPose{
	private:
		ros::NodeHandle nh;
		/*subscribe*/
		ros::Subscriber sub_inipose;
		ros::Subscriber sub_bias;
		ros::Subscriber sub_imu;
		ros::Subscriber sub_slam;
		ros::Subscriber sub_walls;
		/*const*/
		const int num_state = 3;
		/*objects*/
		tf::Quaternion q_pose;
		tf::Quaternion q_pose_last_at_slamcall;
		Eigen::MatrixXd X;
		Eigen::MatrixXd P;
		sensor_msgs::Imu bias;
		/*flags*/
		bool inipose_is_available = false;
		bool bias_is_available = false;
		bool first_callback_imu = true;
		/*time*/
		ros::Time time_imu_now;
		ros::Time time_imu_last;
	public:
		EKFPose();
		void CallbackInipose(const geometry_msgs::QuaternionConstPtr& msg);
		void CallbackBias(const sensor_msgs::ImuConstPtr& msg);
		void CallbackImu(const sensor_msgs::ImuConstPtr& msg);
		void PredictionImu(sensor_msgs::Imu imu, double dt);
		void CallbackSlam(const geometry_msgs::PoseStampedConstPtr& msg);
		void CallbackWalls(const sensor_msgs::PointCloud2ConstPtr& msg);
};

EKFPose::EKFPose()
{
	sub_inipose = nh.subscribe("/initial_pose", 1, &EKFPose::CallbackInipose, this);
	sub_bias = nh.subscribe("/imu_bias", 1, &EKFPose::CallbackBias, this);
	sub_imu = nh.subscribe("/imu/data", 1, &EKFPose::CallbackImu, this);
	sub_slam = nh.subscribe("/lsd_slam/pose", 1, &EKFPose::CallbackSlam, this);
	sub_walls = nh.subscribe("/g_and_walls", 1, &EKFPose::CallbackWalls, this);
	X = Eigen::MatrixXd::Constant(num_state, 1, 0.0);
	P = 1.0e-10*Eigen::MatrixXd::Identity(num_state, num_state);
}

void EKFPose::CallbackInipose(const geometry_msgs::QuaternionConstPtr& msg)
{
	if(!inipose_is_available){
		quaternionMsgToTF(*msg, q_pose);
		q_pose.normalize();
		q_pose_last_at_slamcall = q_pose;
		tf::Matrix3x3(q_pose).getRPY(X(0, 0), X(1, 0), X(2, 0));
		inipose_is_available = true;
		std::cout << "inipose_is_available = " << inipose_is_available << std::endl;
		std::cout << "initial pose = " << std::endl << X << std::endl;
	}
}

void EKFPose::CallbackBias(const sensor_msgs::ImuConstPtr& msg)
{
	if(!bias_is_available){
		bias = *msg;
		bias_is_available = true;
	}
}

void EKFPose::CallbackImu(const sensor_msgs::ImuConstPtr& msg)
{
	time_imu_now = ros::Time::now();
	double dt = (time_imu_now - time_imu_last).toSec();
	time_imu_last = time_now;
	if(first_callback_imu)	dt = 0.0;
	else if(inipose_is_available)	PredictionImu(*msg, dt);
	first_callback_imu = false;
}

void EKFPose::PredictionImu(sensor_msgs::Imu imu, double dt)
{
}

void EKFPose::CallbackSlam(const geometry_msgs::PoseStampedConstPtr& msg)
{
}

void EKFPose::CallbackWalls(const sensor_msgs::PointCloud2ConstPtr& msg)
{
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ekf_pose");
	std::cout << "E.K.F. POSE" << std::endl;
	
	EKFPose ekf_pose;
	ros::spin();
}
