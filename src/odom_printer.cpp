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
		/*objects*/
		tf::TransformListener listener;
		tf::Quaternion q_true_ini_position;
		tf::Quaternion q_true_ini_pose;
		tf::Quaternion q_true_cur_position;
		tf::Quaternion q_true_cur_pose;
		double est_ini_rpy[3] = {};
		/*flags*/
		bool mocap_is_available = false;
	public:
		OdomPrinter();
		void GetMocapTF(void);
		void CallbackInipose(const geometry_msgs::QuaternionConstPtr& msg);
		void CallbackOdom1(const nav_msgs::OdometryConstPtr& msg);
		void CallbackOdom2(const nav_msgs::OdometryConstPtr& msg);
		void CallbackOdom3(const nav_msgs::OdometryConstPtr& msg);
		void CallbackOdom4(const nav_msgs::OdometryConstPtr& msg);
		void Print(nav_msgs::Odometry odom);
};

OdomPrinter::OdomPrinter()
{
	sub_inipose = nh.subscribe("/initial_pose", 1, &OdomPrinter::CallbackOdom1, this);
	sub_odom1 = nh.subscribe("/combined_odometry", 1, &OdomPrinter::CallbackOdom1, this);
	sub_odom2 = nh.subscribe("/gyrodometry", 1, &OdomPrinter::CallbackOdom2, this);
	sub_odom3 = nh.subscribe("/loamvelodyne_odometry", 1, &OdomPrinter::CallbackOdom3, this);
	sub_odom4 = nh.subscribe("/lsdslam_odometry", 1, &OdomPrinter::CallbackOdom4, this);
}

void OdomPrinter::GetMocapTF(void)
{
	tf::StampedTransform transform;
	try{
		listener.lookupTransform("/vicon/infant/infant", "/world", ros::Time(0), transform);
		q_true_cur_position = tf::Quaternion(
				transform.getOrigin().x(),
				transform.getOrigin().y(),
				transform.getOrigin().z(),
				0.0);
		q_true_cur_pose = transform.getRotation();
		if(!mocap_is_available){
			q_true_ini_position = q_true_cur_position;
			q_true_ini_pose = q_true_cur_pose;
		}
		q_true_cur_position = tf::Quaternion(
			q_true_cur_position.x() - q_true_ini_position.x(),
			q_true_cur_position.y() - q_true_ini_position.y(),
			q_true_cur_position.z() - q_true_ini_position.z(),
			0.0);
		q_true_cur_position = q_true_ini_pose.inverse()*q_true_cur_position*q_true_ini_pose;
		q_true_cur_pose = q_true_ini_pose.inverse()*q_true_cur_pose;
		mocap_is_available = true;
	}
	catch(tf::TransformException &ex){
		ROS_ERROR("%s",ex.what());
		ros::Duration(1.0).sleep();
	}
}

void OdomPrinter::CallbackInipose(const geometry_msgs::QuaternionConstPtr& msg)
{
	tf::Quaternion q_est_ini_pose;
	quaternionMsgToTF(*msg, q_est_ini_pose);
	tf::Matrix3x3(q_est_ini_pose).getRPY(est_ini_rpy[0], est_ini_rpy[1], est_ini_rpy[2]);
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

void OdomPrinter::Print(nav_msgs::Odometry odom)
{
	double est_cur_rpy[3];
	tf::Quaternion q_est_cur_pose;
	quaternionMsgToTF(odom.pose.pose.orientation, q_est_cur_pose);
	tf::Matrix3x3(q_est_cur_pose).getRPY(est_cur_rpy[0], est_cur_rpy[1], est_cur_rpy[2]);

	double error_xyz[3];
	double error_rpy[3];

	std::cout << "--- " << odom.child_frame_id << "  ---" << std::endl;
	if(mocap_is_available){
		double true_cur_rpy[3];
		tf::Matrix3x3(q_true_cur_pose).getRPY(true_cur_rpy[0], true_cur_rpy[1], true_cur_rpy[2]);
		error_xyz[0] = odom.pose.pose.position.x - q_true_cur_position.x();
		error_xyz[1] = odom.pose.pose.position.y - q_true_cur_position.y();
		error_xyz[2] = odom.pose.pose.position.z - q_true_cur_position.z();
		for(int i=0;i<3;i++)	error_rpy[i] = est_cur_rpy[i] - true_cur_rpy[i];

		std::cout << "true_position[m]: (" << q_true_cur_position.x() << ", " << q_true_cur_position.y() << ", " << q_true_cur_position.z() << ")" << std::endl;
		std::cout << "true_pose[m]: (" << true_cur_rpy[0] << ", " << true_cur_rpy[1] << ", " << true_cur_rpy[2] << ")" << std::endl;
	}
	else{
		error_xyz[0] = odom.pose.pose.position.x;
		error_xyz[1] = odom.pose.pose.position.y;
		error_xyz[2] = odom.pose.pose.position.z;
		for(int i=0;i<3;i++)	error_rpy[i] = est_cur_rpy[i] - est_ini_rpy[i];
	}
	double error_euc_dist = 0.0;
	for(int i=0;i<3;i++)	error_euc_dist += error_xyz[i]*error_xyz[i];
	error_euc_dist = sqrt(error_euc_dist);

	std::cout << "est xyz[m]        :(" << odom.pose.pose.position.x << ", " << odom.pose.pose.position.y << ", " << odom.pose.pose.position.z << ")" << std::endl;
	std::cout << "error xyz[m]      :(" << error_xyz[0] << ", " << error_xyz[1] << ", " << error_xyz[2] << ")" << std::endl;
	std::cout << "error Euc.dist.[m]:" << error_euc_dist << std::endl;
	std::cout << "est rpy[deg]      :(" << est_cur_rpy[0]/M_PI*180.0 << ", " << est_cur_rpy[1]/M_PI*180.0 << ", " << est_cur_rpy[2]/M_PI*180.0 << ")  " << std::endl;
	std::cout << "error rpy[deg]    :(" << error_rpy[0]/M_PI*180.0 << ", " << error_rpy[1]/M_PI*180.0 << ", " << error_rpy[2]/M_PI*180.0 << ")" << std::endl;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "odom_printer");

	OdomPrinter odom_printer;

	// ros::spin();
	ros::Rate loop_rate(10);
	while(ros::ok()){
		std::cout << "========================" << std::endl;
		odom_printer.GetMocapTF();
		ros::spinOnce();
		loop_rate.sleep();
	}
}
