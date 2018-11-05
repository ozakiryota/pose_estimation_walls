#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/tf.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
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
		/*publish*/
		ros::Publisher pub;
		/*const*/
		const int num_state = 3;
		/*objects*/
		tf::Quaternion q_pose;
		tf::Quaternion q_pose_last_at_slamcall;
		Eigen::MatrixXd X;
		Eigen::MatrixXd P;
		sensor_msgs::Imu bias;
		tf::Quaternion q_slam_now;
		tf::Quaternion q_slam_last;
		/*flags*/
		bool inipose_is_available = false;
		bool bias_is_available = false;
		bool first_callback_imu = true;
		/*counter*/
		int count_usingwalls = 0;
		int count_slam = 0;
		/*time*/
		ros::Time time_imu_now;
		ros::Time time_imu_last;
		/*for yaw estimation with walls*/
		tf::Quaternion q_pose_last_at_wallscall;
		pcl::PointCloud<pcl::PointXYZ>::Ptr walls_now {new pcl::PointCloud<pcl::PointXYZ>};
		pcl::PointCloud<pcl::PointXYZ>::Ptr walls_last {new pcl::PointCloud<pcl::PointXYZ>};
	public:
		EKFPose();
		void CallbackInipose(const geometry_msgs::QuaternionConstPtr& msg);
		void CallbackBias(const sensor_msgs::ImuConstPtr& msg);
		void CallbackIMU(const sensor_msgs::ImuConstPtr& msg);
		void PredictionIMU(sensor_msgs::Imu imu, double dt);
		void CallbackSLAM(const geometry_msgs::PoseStampedConstPtr& msg);
		void ObservationSLAM(void);
		void CallbackWalls(const sensor_msgs::PointCloud2ConstPtr& msg);
		void ObservationWalls(pcl::PointNormal g_vector);
		bool YawEstimationWalls(double& yaw_walls);
		void Publisher();
};

EKFPose::EKFPose()
{
	sub_inipose = nh.subscribe("/initial_pose", 1, &EKFPose::CallbackInipose, this);
	sub_bias = nh.subscribe("/imu_bias", 1, &EKFPose::CallbackBias, this);
	sub_imu = nh.subscribe("/imu/data", 1, &EKFPose::CallbackIMU, this);
	sub_slam = nh.subscribe("/lsd_slam/pose", 1, &EKFPose::CallbackSLAM, this);
	sub_walls = nh.subscribe("/g_and_walls", 1, &EKFPose::CallbackWalls, this);
	pub = nh.advertise<geometry_msgs::PoseStamped>("/pose_ekf", 1);
	q_pose = tf::Quaternion(0.0, 0.0, 0.0, 1.0);
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

void EKFPose::CallbackIMU(const sensor_msgs::ImuConstPtr& msg)
{
	time_imu_now = ros::Time::now();
	double dt = (time_imu_now - time_imu_last).toSec();
	time_imu_last = time_imu_now;
	if(first_callback_imu)	dt = 0.0;
	else if(inipose_is_available)	PredictionIMU(*msg, dt);
	
	Publisher();

	first_callback_imu = false;
}

void EKFPose::PredictionIMU(sensor_msgs::Imu imu, double dt)
{
	double roll = X(0, 0);
	double pitch = X(1, 0);
	double yaw = X(2, 0);

	double delta_r = imu.angular_velocity.x*dt;
	double delta_p = imu.angular_velocity.y*dt;
	double delta_y = imu.angular_velocity.z*dt;
	if(bias_is_available){
		delta_r -= bias.angular_velocity.x*dt;
		delta_p -= bias.angular_velocity.y*dt;
		delta_y -= bias.angular_velocity.z*dt;
	}

	Eigen::MatrixXd F(num_state, 1);
	F <<	roll + (delta_r + sin(roll)*tan(pitch)*delta_p + cos(roll)*tan(pitch)*delta_y),
			pitch + (cos(roll)*delta_p - sin(roll)*delta_y),
			yaw + (sin(roll)/cos(pitch)*delta_p + cos(roll)/cos(pitch)*delta_y);
	double dfdx[num_state][num_state];
	dfdx[0][0] = 1.0 + (cos(roll)*tan(pitch)*delta_p - sin(roll)*tan(pitch)*delta_y);
	dfdx[0][1] = (sin(roll)/cos(pitch)/cos(pitch)*delta_p + cos(roll)/cos(pitch)/cos(pitch)*delta_y);
	dfdx[0][2] = 0.0;
	dfdx[1][0] = (-sin(roll)*delta_p - cos(roll)*delta_y);
	dfdx[1][1] = 1.0;
	dfdx[1][2] = 0.0;
	dfdx[2][0] = (cos(roll)/cos(pitch)*delta_p - sin(roll)/cos(pitch)*delta_y);
	dfdx[2][1] = (-sin(roll)/sin(pitch)*delta_p - cos(roll)/sin(pitch)*delta_y);
	dfdx[2][2] = 1.0;
	Eigen::MatrixXd jF(num_state, num_state);
	for(int i=0;i<num_state;i++){
		for(int j=0;j<num_state;j++)    jF(i, j) = dfdx[i][j];
	}
	const double sigma = 1.0e-1;
	Eigen::MatrixXd Q = sigma*Eigen::MatrixXd::Identity(num_state, num_state);
	Q <<	1.0e-1,	0, 0,
	 		0,	1.0e-1,	0,
			0,	0,	5.0e+5;
	
	// X = F;
	// for(int i=0;i<3;i++){
	// 	if(X(i, 0)>M_PI)	X(i, 0) -= 2.0*M_PI;
	// 	if(X(i, 0)<-M_PI)	X(i, 0) += 2.0*M_PI;
	// }
	tf::Quaternion q_relative_rotation = tf::createQuaternionFromRPY(delta_r, delta_p, delta_y);
	q_pose = q_pose*q_relative_rotation;
	q_pose.normalize();
	tf::Matrix3x3(q_pose).getRPY(X(0, 0), X(1, 0), X(2, 0));
	// q_pose = tf::createQuaternionFromRPY(X(0, 0), X(1, 0), X(2, 0));

	P = jF*P*jF.transpose() + Q;
}

void EKFPose::CallbackSLAM(const geometry_msgs::PoseStampedConstPtr& msg)
{
	q_slam_now = tf::Quaternion(msg->pose.orientation.z, -msg->pose.orientation.x, -msg->pose.orientation.y, msg->pose.orientation.w);	//fix for LSD-SLAM
	q_slam_now.normalize();
	
	if(inipose_is_available)	ObservationSLAM();
	
	q_slam_last = q_slam_now;
	q_pose_last_at_slamcall = q_pose;
	
	Publisher();
}

void EKFPose::ObservationSLAM(void)
{	
	const int num_obs = 3;
		
	tf::Quaternion q_relative_rotation = q_slam_last.inverse()*q_slam_now;
	q_relative_rotation.normalize();
	q_pose = q_pose_last_at_slamcall*q_relative_rotation;
	q_pose.normalize();
		
	Eigen::MatrixXd Z(num_obs, 1);
	tf::Matrix3x3(q_pose).getRPY(Z(0, 0), Z(1, 0), Z(2, 0));
	Eigen::MatrixXd H(num_obs, num_state);
	H <<	1,	0,	0,
			0,	1,	0,
			0, 	0,	1;
	Eigen::MatrixXd jH(num_obs, num_state);
	jH <<	1,	0,	0,
			0,	1,	0,
			0,	0,	1;
	Eigen::MatrixXd R(num_obs, num_obs);
	const double sigma = 5.0e+5;
	R = sigma*Eigen::MatrixXd::Identity(num_obs, num_obs);
	R <<	5.0e+2,	0, 0,
	  		0,	5.0e+2,	0,
			0,	0,	1.0e-2;
	Eigen::MatrixXd Y(num_obs, 1);
	Eigen::MatrixXd S(num_obs, num_obs);
	Eigen::MatrixXd K(num_state, num_obs);
	Eigen::MatrixXd I = Eigen::MatrixXd::Identity(num_state, num_state);

	Y = Z - H*X;
	for(int i=0;i<3;i++){
		if(Y(i, 0)>M_PI)	Y(i, 0) -= 2.0*M_PI;
		else if(Y(i, 0)<-M_PI)	Y(i, 0) += 2.0*M_PI;
	}
	S = jH*P*jH.transpose() + R;
	K = P*jH.transpose()*S.inverse();
	X = X + K*Y;
	P = (I - K*jH)*P;

	for(int i=0;i<3;i++){
		if(X(i, 0)>M_PI)	X(i, 0) -= 2.0*M_PI;
		else if(X(i, 0)<-M_PI)	X(i, 0) += 2.0*M_PI;
	}

	count_slam++;
	if(count_slam%500==0){
		std::cout << count_slam << ": CALLBACK SLAM" << std::endl;
		std::cout << "Y = " << std::endl << Y << std::endl;
		std::cout << "K*Y = " << std::endl << K*Y << std::endl;
		std::cout << "P = " << std::endl << P << std::endl;
	}
	
	q_pose = tf::createQuaternionFromRPY(X(0, 0), X(1, 0), X(2, 0));
}

void EKFPose::CallbackWalls(const sensor_msgs::PointCloud2ConstPtr& msg)
{
	pcl::PointCloud<pcl::PointNormal>::Ptr tmp_normals (new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointNormal g_vector;
	pcl::fromROSMsg(*msg, *tmp_normals);
	walls_now->points.clear();
	for(size_t i=0;i<tmp_normals->points.size();i++){
		if(i==0)	g_vector = tmp_normals->points[i];
		else{
			pcl::PointXYZ tmp_point;
			tmp_point.x = tmp_normals->points[i].normal_x;
			tmp_point.y = tmp_normals->points[i].normal_y;
			tmp_point.z = tmp_normals->points[i].normal_z;
			walls_now->points.push_back(tmp_point);
		}
	}
	if(inipose_is_available)	ObservationWalls(g_vector);
	q_pose_last_at_wallscall = q_pose;
	*walls_last = *walls_now;

	Publisher();
}

void EKFPose::ObservationWalls(pcl::PointNormal g_vector)
{
	const int num_obs = 3;
		
	count_usingwalls++;
	std::cout << count_usingwalls << ": CALLBACK USINGWALLS" << std::endl;
	
	const double g = -9.80665;
	double gx = g_vector.normal_x*g; 
	double gy = g_vector.normal_y*g; 
	double gz = g_vector.normal_z*g; 

	Eigen::MatrixXd Z(num_obs, 1);
	Z <<	atan2(gy, gz),
	  		atan2(-gx, sqrt(gy*gy + gz*gz)),
			X(2, 0);
	double yaw_walls;
	if(YawEstimationWalls(yaw_walls))	Z <<	atan2(gy, gz),
	  											atan2(-gx, sqrt(gy*gy + gz*gz)),
												yaw_walls;
	Eigen::MatrixXd H(num_obs, num_state);
	H <<	1,	0,	0,
			0,	1,	0,
			0, 	0,	1;
	Eigen::MatrixXd jH(num_obs, num_state);
	jH <<	1,	0,	0,
			0,	1,	0,
			0,	0,	1;
	Eigen::MatrixXd R(num_obs, num_obs);
	const double sigma = 1.0e+1;
	R = sigma*Eigen::MatrixXd::Identity(num_obs, num_obs);
	R <<	1.0e+1,	0, 0,
	  		0,	1.0e+1,	0,
			0,	0,	1.0e+1;
	Eigen::MatrixXd Y(num_obs, 1);
	Eigen::MatrixXd S(num_obs, num_obs);
	Eigen::MatrixXd K(num_state, num_obs);
	Eigen::MatrixXd I = Eigen::MatrixXd::Identity(num_state, num_state);

	Y = Z - H*X;
	// for(int i=0;i<num_obs;i++)	Y(i, 0) = atan2(sin(Y(i, 0)), cos(Y(i, 0)));
	for(int i=0;i<3;i++){
		if(Y(i, 0)>M_PI)	Y(i, 0) -= 2.0*M_PI;
		if(Y(i, 0)<-M_PI)	Y(i, 0) += 2.0*M_PI;
	}
	S = jH*P*jH.transpose() + R;
	K = P*jH.transpose()*S.inverse();
	// K(2, 0) = 0.0;	//temporary repair
	// K(2, 1) = 0.0;	//temporary repair
	// K(2, 2) = 0.0;	//temporary repair
	X = X + K*Y;
	P = (I - K*jH)*P;
		
	for(int i=0;i<3;i++){
		if(X(i, 0)>M_PI)	X(i, 0) -= 2.0*M_PI;
		if(X(i, 0)<-M_PI)	X(i, 0) += 2.0*M_PI;
	}

	std::cout << "Y = " << std::endl << Y << std::endl;
	std::cout << "K*Y = " << std::endl << K*Y << std::endl;
	
	q_pose = tf::createQuaternionFromRPY(X(0, 0), X(1, 0), X(2, 0));
}

bool EKFPose::YawEstimationWalls(double& yaw_walls)
{
	if(walls_last->points.empty()){
		*walls_last = *walls_now;
		return false;
	}
	else{
		/*rotate wall points*/
		pcl::PointCloud<pcl::PointXYZ>::Ptr walls_now_rotated (new pcl::PointCloud<pcl::PointXYZ>);
		tf::Quaternion relative_rotation = q_pose*q_pose_last_at_wallscall.inverse();
		relative_rotation.normalize();
		Eigen::Quaternionf rotation(relative_rotation.w(), relative_rotation.x(), relative_rotation.y(), relative_rotation.z());
		Eigen::Vector3f offset(0.0, 0.0, 0.0);
		pcl::transformPointCloud(*walls_now, *walls_now_rotated, offset, rotation);
		/*matching*/
		int k = 1;
		pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
		std::vector<int> pointIdxNKNSearch(k);
		std::vector<float> pointNKNSquaredDistance(k);
		kdtree.setInputCloud(walls_now_rotated);
		const double threshold_matching_distance = 0.4;
		std::vector<double> list_yawrate;
		for(size_t i=0;i<walls_last->points.size();i++){
			if(kdtree.nearestKSearch(walls_last->points[i], k, pointIdxNKNSearch, pointNKNSquaredDistance)<=0)	std::cout << "kdtree error" << std::endl;
			if(sqrt(pointNKNSquaredDistance[0])<threshold_matching_distance){
				tf::Quaternion q1(
						walls_now->points[pointIdxNKNSearch[0]].x,
						walls_now->points[pointIdxNKNSearch[0]].y,
						walls_now->points[pointIdxNKNSearch[0]].z,
						1.0);
				tf::Quaternion q2(
						walls_last->points[i].x,
						walls_last->points[i].y,
						walls_last->points[i].z,
						1.0);
				tf::Quaternion relative_rotation_ = (q_pose_last_at_wallscall*q2)*(q_pose_last_at_wallscall*q1).inverse();
				relative_rotation_.normalize();
				double roll_rate, pitch_rate, yaw_rate;
				tf::Matrix3x3(relative_rotation_).getRPY(roll_rate, pitch_rate, yaw_rate);
				list_yawrate.push_back(yaw_rate);
			}
		}
		if(!list_yawrate.empty()){
			double yawrate_ave = 0.0;
			for(size_t i=0;i<list_yawrate.size();i++)	yawrate_ave += list_yawrate[i]/(double)list_yawrate.size();
			std::cout << "estimated yaw with walls" << std::endl;
			double roll, pitch, yaw;
			tf::Matrix3x3(q_pose_last_at_wallscall).getRPY(roll, pitch, yaw);
			yaw_walls = atan2(sin(yaw + yawrate_ave), cos(yaw + yawrate_ave));
			return true;
		}
		else	return false;
	}
}

void EKFPose::Publisher(void)
{
	geometry_msgs::PoseStamped pose_out;
	q_pose.normalize();
	quaternionTFToMsg(q_pose, pose_out.pose.orientation);
	pose_out.header.frame_id = "/odom";
	pose_out.header.stamp = ros::Time::now();
	pub.publish(pose_out);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ekf_pose");
	std::cout << "E.K.F. POSE" << std::endl;
	
	EKFPose ekf_pose;
	ros::spin();
}
