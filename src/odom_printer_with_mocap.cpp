#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>

class OdomPrinter{
	private:
		/*node handle*/
		ros::NodeHandle nh;
		/*subscribe*/
		ros::Subscriber sub_inipose;
		ros::Subscriber sub_odom1;
		ros::Subscriber sub_odom2;
		ros::Subscriber sub_odom3;
		ros::Subscriber sub_odom4;
		ros::Subscriber sub_odom5;
		/*objects*/
		tf::TransformListener listener;
		tf::Quaternion q_est_ini_pose = {0.0, 0.0, 0.0, 1.0};
		tf::Quaternion q_mocap_ini_position;
		tf::Quaternion q_mocap_ini_pose;
		tf::Quaternion q_mocap_relative_position;
		tf::Quaternion q_mocap_relative_pose;
		double mocap_relative_rpy[3];
		double error_xyz_odom1[3];
		double error_xyz_odom2[3];
		double error_xyz_odom3[3];
		double error_xyz_odom4[3];
		double error_xyz_odom5[3];
		double error_rpy_odom1[3];
		double error_rpy_odom2[3];
		double error_rpy_odom3[3];
		double error_rpy_odom4[3];
		double error_rpy_odom5[3];
		double relative_rpy_odom1[3];
		double relative_rpy_odom2[3];
		double relative_rpy_odom3[3];
		double relative_rpy_odom4[3];
		double relative_rpy_odom5[3];
		/*flags*/
		bool mocap_inipose_is_available = false;
		bool first_callback_odom = true;
		/*time*/
		ros::Time time_start;
	public:
		OdomPrinter();
		void GetMocapTF(void);
		void CallbackInipose(const geometry_msgs::QuaternionConstPtr& msg);
		void CallbackOdom1(const nav_msgs::OdometryConstPtr& msg);
		void CallbackOdom2(const nav_msgs::OdometryConstPtr& msg);
		void CallbackOdom3(const nav_msgs::OdometryConstPtr& msg);
		void CallbackOdom4(const nav_msgs::OdometryConstPtr& msg);
		void CallbackOdom5(const nav_msgs::OdometryConstPtr& msg);
		void CalculateError(nav_msgs::Odometry odom, double (&error_xyz)[3], double (&error_rpy)[3], double (&est_relative_rpy)[3]);
		void Print(void);
};

OdomPrinter::OdomPrinter()
{
	sub_inipose = nh.subscribe("/initial_pose", 1, &OdomPrinter::CallbackInipose, this);
	sub_odom1 = nh.subscribe("/combined_odometry", 1, &OdomPrinter::CallbackOdom1, this);
	sub_odom2 = nh.subscribe("/gyrodometry", 1, &OdomPrinter::CallbackOdom2, this);
	sub_odom3 = nh.subscribe("/loamvelodyne_odometry", 1, &OdomPrinter::CallbackOdom3, this);
	sub_odom4 = nh.subscribe("/lsdslam_odometry", 1, &OdomPrinter::CallbackOdom4, this);
	sub_odom5 = nh.subscribe("/imu_odometry", 1, &OdomPrinter::CallbackOdom5, this);
}

void OdomPrinter::GetMocapTF(void)
{
	tf::StampedTransform transform;
	try{
		// listener.lookupTransform("/vicon/infant/infant", "/world", ros::Time(0), transform);
		listener.lookupTransform("/world", "/vicon/infant/infant", ros::Time(0), transform);
		tf::Quaternion q_raw_position(
			transform.getOrigin().x(),
			transform.getOrigin().y(),
			transform.getOrigin().z(),
			0.0);
		tf::Quaternion q_raw_pose = transform.getRotation();
		if(!mocap_inipose_is_available){
			q_mocap_ini_position = q_raw_position;
			q_mocap_ini_pose = q_raw_pose;

			double ini_rpy[3];
			tf::Matrix3x3(q_mocap_ini_pose).getRPY(ini_rpy[0], ini_rpy[1], ini_rpy[2]);
			// std::cout << "ini xyz[m]:(" << q_mocap_ini_position.x() << ", " << q_mocap_ini_position.y() << ", " << q_mocap_ini_position.z() << ")" << std::endl;
			// std::cout << "ini rpy[deg]:(" << ini_rpy[0]/M_PI*180.0 << ", " << ini_rpy[1]/M_PI*180.0 << ", " << ini_rpy[2]/M_PI*180.0 << ")" << std::endl;
		}
		q_mocap_relative_position = tf::Quaternion(
			q_raw_position.x() - q_mocap_ini_position.x(),
			q_raw_position.y() - q_mocap_ini_position.y(),
			q_raw_position.z() - q_mocap_ini_position.z(),
			0.0);
		q_mocap_relative_position = q_mocap_ini_pose.inverse()*q_mocap_relative_position*q_mocap_ini_pose;
		q_mocap_relative_pose = q_mocap_ini_pose.inverse()*q_raw_pose;
		q_mocap_relative_pose.normalize();
		tf::Matrix3x3(q_mocap_relative_pose).getRPY(mocap_relative_rpy[0], mocap_relative_rpy[1], mocap_relative_rpy[2]);

		mocap_inipose_is_available = true;
	}
	catch(tf::TransformException &ex){
		ROS_ERROR("%s",ex.what());
		ros::Duration(0.1).sleep();
	}
}

void OdomPrinter::CallbackInipose(const geometry_msgs::QuaternionConstPtr& msg)
{
	quaternionMsgToTF(*msg, q_est_ini_pose);
}

void OdomPrinter::CallbackOdom1(const nav_msgs::OdometryConstPtr& msg)
{
	CalculateError(*msg, error_xyz_odom1, error_rpy_odom1, relative_rpy_odom1);
	if(first_callback_odom){
		time_start = ros::Time::now();
		first_callback_odom = false;
	}
}

void OdomPrinter::CallbackOdom2(const nav_msgs::OdometryConstPtr& msg)
{
	CalculateError(*msg, error_xyz_odom2, error_rpy_odom2, relative_rpy_odom2);
	if(first_callback_odom){
		time_start = ros::Time::now();
		first_callback_odom = false;
	}
}

void OdomPrinter::CallbackOdom3(const nav_msgs::OdometryConstPtr& msg)
{
	CalculateError(*msg, error_xyz_odom3, error_rpy_odom3, relative_rpy_odom3);
	if(first_callback_odom){
		time_start = ros::Time::now();
		first_callback_odom = false;
	}
}

void OdomPrinter::CallbackOdom4(const nav_msgs::OdometryConstPtr& msg)
{
	CalculateError(*msg, error_xyz_odom4, error_rpy_odom4, relative_rpy_odom4);
	if(first_callback_odom){
		time_start = ros::Time::now();
		first_callback_odom = false;
	}
}

void OdomPrinter::CallbackOdom5(const nav_msgs::OdometryConstPtr& msg)
{
	CalculateError(*msg, error_xyz_odom5, error_rpy_odom5, relative_rpy_odom5);
	if(first_callback_odom){
		time_start = ros::Time::now();
		first_callback_odom = false;
	}
}

void OdomPrinter::CalculateError(nav_msgs::Odometry odom, double (&error_xyz)[3], double (&error_rpy)[3], double (&est_relative_rpy)[3])
{
	tf::Quaternion q_raw_position = tf::Quaternion(
		odom.pose.pose.position.x,
		odom.pose.pose.position.y,
		odom.pose.pose.position.z,
		0.0);
	tf::Quaternion q_est_relative_position = q_est_ini_pose.inverse()*q_raw_position*q_est_ini_pose;

	double raw_rpy[3];
	tf::Quaternion q_raw_pose;
	quaternionMsgToTF(odom.pose.pose.orientation, q_raw_pose);
	tf::Matrix3x3(q_raw_pose).getRPY(raw_rpy[0], raw_rpy[1], raw_rpy[2]);

	// double est_relative_rpy[3];
	tf::Quaternion q_est_relative_pose = q_est_ini_pose.inverse()*q_raw_pose;
	tf::Matrix3x3(q_est_relative_pose).getRPY(est_relative_rpy[0], est_relative_rpy[1], est_relative_rpy[2]);

	// double error_xyz[3];
	error_xyz[0] = q_est_relative_position.x() - q_mocap_relative_position.x();
	error_xyz[1] = q_est_relative_position.y() - q_mocap_relative_position.y();
	error_xyz[2] = q_est_relative_position.z() - q_mocap_relative_position.z();
	// double error_rpy[3];	//[deg]
	// for(size_t i=0;i<3;i++)	error_rpy[i] = atan2(sin(est_relative_rpy[i] - mocap_relative_rpy[i]), cos(est_relative_rpy[i] - mocap_relative_rpy[i]))/M_PI*180.0;
	for(size_t i=0;i<3;i++){
		error_rpy[i] = est_relative_rpy[i] - mocap_relative_rpy[i];
		if(error_rpy[i]>M_PI)	error_rpy[i] -= 2.0*M_PI;
		if(error_rpy[i]<-M_PI)	error_rpy[i] += 2.0*M_PI;
	}

	// std::cout << "--- " << odom.child_frame_id << "  ---" << std::endl;
	// std::cout << "raw xyz[m]:(" << q_raw_position.x() << ", " << q_raw_position.y() << ", " << q_raw_position.z() << ")" << std::endl;
	// std::cout << "raw rpy[deg]:(" << raw_rpy[0]/M_PI*180.0 << ", " << raw_rpy[1]/M_PI*180.0 << ", " << raw_rpy[2]/M_PI*180.0 << ")" << std::endl;
	// std::cout << "relative xyz[m]:(" << q_est_relative_position.x() << ", " << q_est_relative_position.y() << ", " << q_est_relative_position.z() << ")" << std::endl;
	// std::cout << "relative rpy[rad]:(" << est_relative_rpy[0] << ", " << est_relative_rpy[1] << ", " << est_relative_rpy[2] << ")" << std::endl;
	// std::cout << "error xyz[m]:(" << error_xyz[0] << ", " << error_xyz[1] << ", " << error_xyz[2] << ")" << std::endl;
	// std::cout << "error rpy[deg]:(" << error_rpy[0] << ", " << error_rpy[1] << ", " << error_rpy[2] << ")" << std::endl;
}

void OdomPrinter::Print(void)
{
	// for(size_t i=0;i<3;i++)	std::cout << fabs(error_xyz_odom1[i]) << ", ";
	// for(size_t i=0;i<3;i++)	std::cout << fabs(error_xyz_odom2[i]) << ", ";
	// for(size_t i=0;i<3;i++)	std::cout << fabs(error_xyz_odom5[i]) << ", ";

	// for(size_t i=0;i<3;i++)	std::cout << mocap_relative_rpy[i] << ", ";
	// for(size_t i=0;i<3;i++)	std::cout << relative_rpy_odom1[i] << ", ";
	// for(size_t i=0;i<3;i++)	std::cout << relative_rpy_odom2[i] << ", ";
	// for(size_t i=0;i<3;i++)	std::cout << relative_rpy_odom5[i] << ", ";
	// for(size_t i=0;i<3;i++)	std::cout << fabs(error_rpy_odom1[i]) << ", ";
	// for(size_t i=0;i<3;i++)	std::cout << fabs(error_rpy_odom2[i]) << ", ";
	// for(size_t i=0;i<3;i++)	std::cout << fabs(error_rpy_odom5[i]) << ", ";

	std::cout << (ros::Time::now() - time_start).toSec() << ",";
	for(size_t i=0;i<3;i++){
		std::cout << mocap_relative_rpy[i]/M_PI*180.0 << ", ";
		std::cout << relative_rpy_odom1[i]/M_PI*180.0 << ", ";
		std::cout << relative_rpy_odom2[i]/M_PI*180.0 << ", ";
		std::cout << relative_rpy_odom5[i]/M_PI*180.0 << ", ";
		std::cout << fabs(error_rpy_odom1[i])/M_PI*180.0 << ", ";
		std::cout << fabs(error_rpy_odom2[i])/M_PI*180.0 << ", ";
		std::cout << fabs(error_rpy_odom5[i])/M_PI*180.0 << ", ";
	}
	std::cout << std::endl;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "odom_printer");

	OdomPrinter odom_printer;

	// std::cout << ",mocap,,,proposed,,,gyro,,,ahrs,,,proposed,,,gyro,,,ahrs" << std::endl;
	// std::cout << "time,,,,proposed,,,gyro,,,ahrs" << std::endl;

	// ros::spin();
	ros::Rate loop_rate(30);
	while(ros::ok()){
		// std::cout << "========================" << std::endl;
		odom_printer.GetMocapTF();
		ros::spinOnce();
		odom_printer.Print();
		loop_rate.sleep();
	}
}
