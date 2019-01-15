#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

class MocapPrinter{
	private:
		/*node handle*/
		ros::NodeHandle nh;
		/*objects*/
		tf::TransformListener listener;
		tf::Quaternion q_ini_position;
		tf::Quaternion q_ini_pose;
		tf::Quaternion q_raw_position;
		tf::Quaternion q_raw_pose;
		tf::Quaternion q_relative_position;
		tf::Quaternion q_relative_pose;
		/*flags*/
		bool inipose_is_available = false;
	public:
		MocapPrinter();
		void GetMocapTF(void);
		void Print(void);
};

MocapPrinter::MocapPrinter()
{
}

void MocapPrinter::GetMocapTF(void)
{
	tf::StampedTransform transform;
	try{
		// listener.lookupTransform("/vicon/infant/infant", "/world", ros::Time(0), transform);
		listener.lookupTransform("/world", "/vicon/infant/infant", ros::Time(0), transform);
		q_raw_position = tf::Quaternion(
			transform.getOrigin().x(),
			transform.getOrigin().y(),
			transform.getOrigin().z(),
			0.0);
		q_raw_pose = transform.getRotation();
		if(!inipose_is_available){
			q_ini_position = q_raw_position;
			q_ini_pose = q_raw_pose;

			double ini_rpy[3];
			tf::Matrix3x3(q_ini_pose).getRPY(ini_rpy[0], ini_rpy[1], ini_rpy[2]);
			std::cout << "ini xyz[m]:(" << q_ini_position.x() << ", " << q_ini_position.y() << ", " << q_ini_position.z() << ")" << std::endl;
			std::cout << "ini rpy[deg]:(" << ini_rpy[0]/M_PI*180.0 << ", " << ini_rpy[1]/M_PI*180.0 << ", " << ini_rpy[2]/M_PI*180.0 << ")" << std::endl;
		}
		q_relative_position = tf::Quaternion(
			q_raw_position.x() - q_ini_position.x(),
			q_raw_position.y() - q_ini_position.y(),
			q_raw_position.z() - q_ini_position.z(),
			0.0);
		q_relative_position = q_ini_pose.inverse()*q_relative_position*q_ini_pose;
		q_relative_pose = q_ini_pose.inverse()*q_raw_pose;
		inipose_is_available = true;

		Print();
	}
	catch(tf::TransformException &ex){
		ROS_ERROR("%s",ex.what());
		ros::Duration(1.0).sleep();
	}
}

void MocapPrinter::Print(void)
{
	double raw_rpy[3];
	tf::Matrix3x3(q_raw_pose).getRPY(raw_rpy[0], raw_rpy[1], raw_rpy[2]);
	double relative_rpy[3];
	tf::Matrix3x3(q_relative_pose).getRPY(relative_rpy[0], relative_rpy[1], relative_rpy[2]);

	std::cout << "raw xyz[m]:(" << q_raw_position.x() << ", " << q_raw_position.y() << ", " << q_raw_position.z() << ")" << std::endl;
	std::cout << "raw rpy[deg]:(" << raw_rpy[0]/M_PI*180.0 << ", " << raw_rpy[1]/M_PI*180.0 << ", " << raw_rpy[2]/M_PI*180.0 << ")" << std::endl;
	std::cout << "relative xyz[m]:(" << q_relative_position.x() << ", " << q_relative_position.y() << ", " << q_relative_position.z() << ")" << std::endl;
	std::cout << "relative rpy[rad]:(" << relative_rpy[0] << ", " << relative_rpy[1] << ", " << relative_rpy[2] << ")" << std::endl;
	std::cout << "relative rpy[deg]:(" << relative_rpy[0]/M_PI*180.0 << ", " << relative_rpy[1]/M_PI*180.0 << ", " << relative_rpy[2]/M_PI*180.0 << ")" << std::endl;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "mocap_printer");

	MocapPrinter mocap_printer;

	// ros::spin();
	ros::Rate loop_rate(10);
	while(ros::ok()){
		std::cout << "========================" << std::endl;
		mocap_printer.GetMocapTF();
		ros::spinOnce();
		loop_rate.sleep();
	}
}
