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
		tf::Quaternion q_ini_pose = {0.0, 0.0, 0.0, 1.0};
	public:
		OdomPrinter();
		void CallbackInipose(const geometry_msgs::QuaternionConstPtr& msg);
		void CallbackOdom1(const nav_msgs::OdometryConstPtr& msg);
		void CallbackOdom2(const nav_msgs::OdometryConstPtr& msg);
		void CallbackOdom3(const nav_msgs::OdometryConstPtr& msg);
		void CallbackOdom4(const nav_msgs::OdometryConstPtr& msg);
		void CallbackOdom5(const nav_msgs::OdometryConstPtr& msg);
		void Print(nav_msgs::Odometry odom);
};

OdomPrinter::OdomPrinter()
{
	sub_inipose = nh.subscribe("/initial_pose", 1, &OdomPrinter::CallbackInipose, this);
	sub_odom1 = nh.subscribe("/combined_odometry", 1, &OdomPrinter::CallbackOdom1, this);
	sub_odom2 = nh.subscribe("/gyrodometry", 1, &OdomPrinter::CallbackOdom2, this);
	sub_odom3 = nh.subscribe("/loamvelodyne_odometry", 1, &OdomPrinter::CallbackOdom3, this);
	sub_odom4 = nh.subscribe("/lsdslam_odometry", 1, &OdomPrinter::CallbackOdom4, this);
	sub_odom5 = nh.subscribe("/ndt_odometry", 1, &OdomPrinter::CallbackOdom5, this);
}

void OdomPrinter::CallbackInipose(const geometry_msgs::QuaternionConstPtr& msg)
{
	quaternionMsgToTF(*msg, q_ini_pose);
}

void OdomPrinter::CallbackOdom1(const nav_msgs::OdometryConstPtr& msg)
{
	Print(*msg);
}

void OdomPrinter::CallbackOdom2(const nav_msgs::OdometryConstPtr& msg)
{
	Print(*msg);
}

void OdomPrinter::CallbackOdom3(const nav_msgs::OdometryConstPtr& msg)
{
	Print(*msg);
}

void OdomPrinter::CallbackOdom4(const nav_msgs::OdometryConstPtr& msg)
{
	Print(*msg);
}

void OdomPrinter::CallbackOdom5(const nav_msgs::OdometryConstPtr& msg)
{
	Print(*msg);
}

void OdomPrinter::Print(nav_msgs::Odometry odom)
{
	tf::Quaternion q_raw_position = tf::Quaternion(
		odom.pose.pose.position.x,
		odom.pose.pose.position.y,
		odom.pose.pose.position.z,
		0.0);
	tf::Quaternion q_relative_position = q_ini_pose.inverse()*q_raw_position*q_ini_pose;

	double raw_rpy[3];
	tf::Quaternion q_raw_pose;
	quaternionMsgToTF(odom.pose.pose.orientation, q_raw_pose);
	tf::Matrix3x3(q_raw_pose).getRPY(raw_rpy[0], raw_rpy[1], raw_rpy[2]);
	double relative_rpy[3];
	tf::Quaternion q_relative_pose = q_ini_pose.inverse()*q_raw_pose;
	tf::Matrix3x3(q_relative_pose).getRPY(relative_rpy[0], relative_rpy[1], relative_rpy[2]);

	std::cout << "--- " << odom.child_frame_id << "  ---" << std::endl;
	std::cout << "raw xyz[m]:(" << q_raw_position.x() << ", " << q_raw_position.y() << ", " << q_raw_position.z() << ")" << std::endl;
	std::cout << "raw rpy[deg]:(" << raw_rpy[0]/M_PI*180.0 << ", " << raw_rpy[1]/M_PI*180.0 << ", " << raw_rpy[2]/M_PI*180.0 << ")" << std::endl;
	std::cout << "relative xyz[m]:(" << q_relative_position.x() << ", " << q_relative_position.y() << ", " << q_relative_position.z() << ")" << std::endl;
	std::cout << "relative rpy[rad]:(" << relative_rpy[0] << ", " << relative_rpy[1] << ", " << relative_rpy[2] << ")" << std::endl;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "odom_printer");

	OdomPrinter odom_printer;

	// ros::spin();
	ros::Rate loop_rate(10);
	while(ros::ok()){
		std::cout << "========================" << std::endl;
		ros::spinOnce();
		loop_rate.sleep();
	}
}
