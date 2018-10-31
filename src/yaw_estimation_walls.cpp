#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/tf.h>

class YawEstimationWalls{
	private:
		ros::NodeHandle nh;
		/*subscribe*/
		ros::Subscriber sub_walls;
		ros::Subscriber sub_pose;
		/*publish*/
		ros::Publisher pub;
		/*pc*/
		pcl::PointNormal g_vector;
		pcl::PointCloud<pcl::PointNormal>::Ptr normals {new pcl::PointCloud<pcl::PointNormal>};
		std::vector<pcl::PointCloud<pcl::PointNormal>> walls;
		/*poses*/
		tf::Quaternion pose_now;
		tf::Quaternion pose_last;
		/*flags*/
		bool first_callback_pose = true;
	public:
		YawEstimationWalls();
		void CallbackWalls(const sensor_msgs::PointCloud2ConstPtr &msg);
		void CallbackPose(const geometry_msgs::PoseStampedConstPtr &msg);
		void RegisterWalls(void);
};

YawEstimationWalls::YawEstimationWalls()
{
	sub_walls = nh.subscribe("/g_and_walls", 1, &YawEstimationWalls::CallbackWalls, this);
	sub_pose = nh.subscribe("/pose_ekf", 1, &YawEstimationWalls::CallbackPose, this);
	pub = nh.advertise<std_msgs::Float64>("/yaw_rate_walls", 1);
}

void YawEstimationWalls::CallbackWalls(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	std::cout << "CALLBACK WALLS" << std::endl;
	normals->points.clear();
		
	pcl::PointCloud<pcl::PointNormal>::Ptr tmp_normals (new pcl::PointCloud<pcl::PointNormal>);
	pcl::fromROSMsg(*msg, *tmp_normals);
	for(size_t i=0;i<tmp_normals->points.size();i++){
		if(i==0)	g_vector = tmp_normals->points[i];
		else	normals->points.push_back(tmp_normals->points[i]);
	}

	if(!first_callback_pose){
	}
}

void YawEstimationWalls::CallbackPose(const geometry_msgs::PoseStampedConstPtr &msg)
{
	std::cout << "CALLBACK POSE" << std::endl;
	quaternionMsgToTF(msg->pose.orientation, pose_now);
	if(first_callback_pose)	pose_last = pose_now;

	first_callback_pose = false;
}

void YawEstimationWalls::RegisterWalls(void)
{
	for(size_t i=0;i<walls.size();i++){
		if(walls[i].size()==2)	walls[i].points.erase(walls[i].points.begin());
	}
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "yaw_estimation_walls");
	
	YawEstimationWalls yaw_estimation_walls;

	ros::spin();
}
