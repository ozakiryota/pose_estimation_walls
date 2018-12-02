#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/LU>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

class ImuInitialAlignment{
	private:
		ros::NodeHandle nh;
		/*subscribe*/
		ros::Subscriber sub;
		/*publish*/
		ros::Publisher pub_inipose;
		ros::Publisher pub_bias;
		tf::TransformBroadcaster tf_broadcaster;
		/*const*/
		const int num_state = 3;
		const double timelimit = 120.0;	//[s]
		/*objects*/
		geometry_msgs::Quaternion initial_pose;
		Eigen::MatrixXd X;
		Eigen::MatrixXd P;
		std::vector<sensor_msgs::Imu> record;
		sensor_msgs::Imu average;
		ros::Time time_started;
		/*flags*/
		bool imu_is_moving = false;
		bool initial_algnment_is_done = false;
	public:
		ImuInitialAlignment();
		void Callback(const sensor_msgs::ImuConstPtr& msg);
		void ComputeAverage(void);
		bool JudgeMoving(void);
		void Prediction(void);
		void Observation(void);
		void Publication(void);
};

ImuInitialAlignment::ImuInitialAlignment()
{
	sub = nh.subscribe("/imu/data", 1, &ImuInitialAlignment::Callback, this);
	pub_inipose = nh.advertise<geometry_msgs::Quaternion>("/initial_pose", 1);
	pub_bias = nh.advertise<sensor_msgs::Imu>("/imu_bias", 1);
	X = Eigen::MatrixXd::Constant(num_state, 1, 0.0);
	P = 100.0*Eigen::MatrixXd::Identity(num_state, num_state);
}

void ImuInitialAlignment::Callback(const sensor_msgs::ImuConstPtr& msg)
{
	if(!initial_algnment_is_done){
		double time;
		try{
			time = (ros::Time::now() - time_started).toSec();
		}
		catch(std::runtime_error& ex) {
			ROS_ERROR("Exception: [%s]", ex.what());
		}
		if(record.size()==0){
			time = 0.0;
			time_started = ros::Time::now();
		}
		else	imu_is_moving = JudgeMoving();
		
		if(imu_is_moving || time>timelimit){
			initial_algnment_is_done = true;
			tf::Quaternion q = tf::createQuaternionFromRPY(X(0, 0), X(1, 0), X(2, 0));
			quaternionTFToMsg(q, initial_pose);
			if(time>timelimit)	std::cout << "time > " << timelimit << "[s]" << std::endl;
			else	std::cout << "Moved at " << time << "[s]" << std::endl;
			std::cout << "initial pose = " << std::endl << X << std::endl;
			std::cout << "bias = " << std::endl << average.angular_velocity.x << std::endl << average.angular_velocity.y << std::endl << average.angular_velocity.z << std::endl;
		}
		else{
			record.push_back(*msg);
			ComputeAverage();
			Prediction();
			Observation();
		}
	}
	else	Publication();
}

void ImuInitialAlignment::ComputeAverage(void)
{
	average.angular_velocity.x = 0.0;
	average.angular_velocity.y = 0.0;
	average.angular_velocity.z = 0.0;
	average.linear_acceleration.x = 0.0;
	average.linear_acceleration.y = 0.0;
	average.linear_acceleration.z = 0.0;
	
	for(size_t i=0;i<record.size();i++){
		average.angular_velocity.x += record[i].angular_velocity.x/(double)record.size();
		average.angular_velocity.y += record[i].angular_velocity.y/(double)record.size();
		average.angular_velocity.z += record[i].angular_velocity.z/(double)record.size();
		average.linear_acceleration.x += record[i].linear_acceleration.x/(double)record.size();
		average.linear_acceleration.y += record[i].linear_acceleration.y/(double)record.size();
		average.linear_acceleration.z += record[i].linear_acceleration.z/(double)record.size();
	}
}

bool ImuInitialAlignment::JudgeMoving(void)
{
	const float threshold_w = 0.03;
	const float threshold_a = 0.20;
	if(fabs(record[record.size()-1].angular_velocity.x - average.angular_velocity.x)>threshold_w){
		std::cout << "Moved-wx" << std::endl;
		return true;
	}
	if(fabs(record[record.size()-1].angular_velocity.y - average.angular_velocity.y)>threshold_w){
		std::cout << "Moved-wy" << std::endl;
		return true;
	}
	if(fabs(record[record.size()-1].angular_velocity.z - average.angular_velocity.z)>threshold_w){
		std::cout << "Moved-wz" << std::endl;
		return true;
	}
	if(fabs(record[record.size()-1].linear_acceleration.x - average.linear_acceleration.x)>threshold_a){
		std::cout << "Moved-ax" << std::endl;
		return true;
	}
	if(fabs(record[record.size()-1].linear_acceleration.y - average.linear_acceleration.y)>threshold_a){
		std::cout << "Moved-ay" << std::endl;
		return true;
	}
	if(fabs(record[record.size()-1].linear_acceleration.z - average.linear_acceleration.z)>threshold_a){
		std::cout << "Moved-az" << std::endl;
		return true;
	}
	return false;
}

void ImuInitialAlignment::Prediction(void)
{
	Eigen::MatrixXd A = Eigen::MatrixXd::Identity(num_state, num_state);
	Eigen::MatrixXd F(num_state, 1);
	Eigen::MatrixXd jF = Eigen::MatrixXd::Identity(num_state, num_state);
	const double sigma = 1.0e-1;
	Eigen::MatrixXd Q = sigma*Eigen::MatrixXd::Identity(num_state, num_state);

	F = A*X;
	X = F;
	P = jF*P*jF.transpose() + Q;
	// std::cout << "X_pre = " << std::endl << X << std::endl;
	// std::cout << "P_pre = " << std::endl << P << std::endl;
}
void ImuInitialAlignment::Observation(void)
{
	const int num_obs = 3;	

	double ax = average.linear_acceleration.x;
	double ay = average.linear_acceleration.y;
	double az = average.linear_acceleration.z;
	double roll = X(0, 0);
	double pitch = X(1, 0);
	double yaw = X(2, 0);
	
	Eigen::MatrixXd Z(num_obs, 1);
	Z <<	atan2(ay, az),
			atan2(-ax, sqrt(ay*ay + az*az)),
			yaw;
	Eigen::MatrixXd H(num_obs, num_state);
	H <<	1,	0,	0,
	  		0,	1,	0,
			0,	0,	1;
	Eigen::MatrixXd jH(num_obs, num_state);
	jH <<	1,	0,	0,
	  		0,	1,	0,
			0,	0,	1;
	const double sigma = 1.0e-1;
	Eigen::MatrixXd R = sigma*Eigen::MatrixXd::Identity(num_obs, num_obs);
	Eigen::MatrixXd Y(num_obs, 1);
	Eigen::MatrixXd S(num_obs, num_obs);
	Eigen::MatrixXd K(num_state, num_obs);
	Eigen::MatrixXd I = Eigen::MatrixXd::Identity(num_state, num_state);

	Y = Z - H*X;
	S = jH*P*jH.transpose() + R;
	K = P*jH.transpose()*S.inverse();
	X = X + K*Y;
	P = (I - K*jH)*P;

	// std::cout << "K*Y = " << std::endl << K*Y << std::endl;
}

void ImuInitialAlignment::Publication(void)
{
	/*publish*/
	pub_inipose.publish(initial_pose);
	pub_bias.publish(average);
	/*tf broadcast*/
    geometry_msgs::TransformStamped transform;
	transform.header.stamp = ros::Time::now();
	transform.header.frame_id = "/odom";
	transform.child_frame_id = "/initial_pose";
	transform.transform.translation.x = 0.0;
	transform.transform.translation.y = 0.0;
	transform.transform.translation.z = 0.0;
	transform.transform.rotation = initial_pose;
	tf_broadcaster.sendTransform(transform);
}

int main(int argc, char**argv)
{
	ros::init(argc, argv, "imu_initial_alignment");

	ImuInitialAlignment imu_initial_alignment;

	ros::spin();
}
