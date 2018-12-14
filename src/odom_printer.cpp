#include <ros/ros.h>
#include <tf/tf.h>
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
		/*objects*/
		double inipose_rpy[3] = {};
	public:
		OdomPrinter();
		void CallbackInipose(const geometry_msgs::QuaternionConstPtr& msg);
		void CallbackOdom1(const nav_msgs::OdometryConstPtr& msg);
		void CallbackOdom2(const nav_msgs::OdometryConstPtr& msg);
		void CallbackOdom3(const nav_msgs::OdometryConstPtr& msg);
		void Print(nav_msgs::Odometry odom);
};

OdomPrinter::OdomPrinter()
{
	sub_inipose = nh.subscribe("/initial_pose", 1, &OdomPrinter::CallbackOdom1, this);
	sub_odom1 = nh.subscribe("/combined_odometry", 1, &OdomPrinter::CallbackOdom1, this);
	sub_odom2 = nh.subscribe("/gyrodometry", 1, &OdomPrinter::CallbackOdom2, this);
	sub_odom3 = nh.subscribe("/loamvelodyne_odometry", 1, &OdomPrinter::CallbackOdom3, this);
}

void OdomPrinter::CallbackInipose(const geometry_msgs::QuaternionConstPtr& msg)
{
	tf::Quaternion q_inipose;
	quaternionMsgToTF(*msg, q_inipose);
	tf::Matrix3x3(q_inipose).getRPY(inipose_rpy[0], inipose_rpy[1], inipose_rpy[2]);
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

void OdomPrinter::Print(nav_msgs::Odometry odom)
{
	tf::Quaternion q_pose;
	quaternionMsgToTF(odom.pose.pose.orientation, q_pose);
	double est_rpy[3];
	tf::Matrix3x3(q_pose).getRPY(est_rpy[0], est_rpy[1], est_rpy[2]);

	std::cout << "--- " << odom.child_frame_id << "  ---" << std::endl;
	std::cout << "position[m]: (" << odom.pose.pose.position.x << ", " << odom.pose.pose.position.y << ", " << odom.pose.pose.position.z << ")" << std::endl;
	std::cout << "Euc.dist.[m]: " << sqrt(odom.pose.pose.position.x*odom.pose.pose.position.x + odom.pose.pose.position.y*odom.pose.pose.position.y + odom.pose.pose.position.z*odom.pose.pose.position.z) << std::endl;
	std::cout << "pose[deg]:   (" << est_rpy[0]/M_PI*180.0 << ", " << est_rpy[1]/M_PI*180.0 << ", " << est_rpy[2]/M_PI*180.0 << ")  " << std::endl;
	std::cout << "error[deg]:  (" << (est_rpy[0] - inipose_rpy[0])/M_PI*180.0 << ", " << (est_rpy[1] - inipose_rpy[1])/M_PI*180.0 << ", " << (est_rpy[2] - inipose_rpy[2])/M_PI*180.0 << ")" << std::endl;
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
