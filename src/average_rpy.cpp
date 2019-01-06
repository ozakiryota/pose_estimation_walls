#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

class AverageRPY{
	private:
		ros::NodeHandle nh;
		/*subscribe*/
		ros::Subscriber sub_rpy;
		/*objects*/
		double rpy_sincosatan[3][3] = {};
		/*time*/
		ros::Time time_started;
		double continue_time = 60;	//[s]
		/*flags*/
		bool first_callback = true;
	public:
		AverageRPY();
		void CallbackRPY(const std_msgs::Float64MultiArrayConstPtr& msg);
};

AverageRPY::AverageRPY()
{
	sub_rpy = nh.subscribe("/rpy_walls", 1, &AverageRPY::CallbackRPY, this);
}

void AverageRPY::CallbackRPY(const std_msgs::Float64MultiArrayConstPtr& msg)
{
	double time;
	if(first_callback){
		time = 0.0;
		time_started = ros::Time::now();
	}
	else{
		try{
			time = (ros::Time::now() - time_started).toSec();
		}
		catch(std::runtime_error& ex) {
			ROS_ERROR("Exception: [%s]", ex.what());
		}
	}
	
	if(time<continue_time){
		for(int i=0;i<3;i++){
			rpy_sincosatan[i][0] += sin(msg->data[i]);
			rpy_sincosatan[i][1] += cos(msg->data[i]);
			rpy_sincosatan[i][2] = atan2(rpy_sincosatan[i][0],rpy_sincosatan[i][1]);
		}
	}
	else{
		std::cout << "time > " << continue_time << " s" << std::endl;
	}
	std::cout << "roll  = " << rpy_sincosatan[0][2]/M_PI*180 << " deg" << std::endl;
	std::cout << "pitch = " << rpy_sincosatan[1][2]/M_PI*180 << " deg" << std::endl;
	std::cout << "yaw   = " << rpy_sincosatan[2][2]/M_PI*180 << " deg" << std::endl;
	std::cout << "----------------------" << std::endl;

	first_callback = false;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "average_rpy");
	
	AverageRPY average_rpy;

	ros::spin();
}
