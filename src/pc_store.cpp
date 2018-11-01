#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/tf.h>
#include <pcl/common/transforms.h>

class PCStore{
	private:
		ros::NodeHandle nh;
		/*subscribe*/
		ros::Subscriber sub_pc;
		ros::Subscriber sub_odom;
		/*viewer*/
		pcl::visualization::PCLVisualizer viewer{"pc_store"};
		/*cloud*/
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_stored {new pcl::PointCloud<pcl::PointXYZ>};
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_now {new pcl::PointCloud<pcl::PointXYZ>};
		/*odom*/
		nav_msgs::Odometry odom_now;
		nav_msgs::Odometry odom_last;
		/*flags*/
		bool first_callback_odom = true;
		bool pc_was_added = false;
		/*time*/
		ros::Time time_odom_now;
		ros::Time time_odom_last;
		/*limit storing*/
		const bool limit_storing = true;
		const size_t limit_num_scans = 100;
		std::vector<size_t> list_num_scanpoints;

	public:
		PCStore();
		void CallbackPC(const sensor_msgs::PointCloud2ConstPtr& msg);
		void CallbackOdom(const nav_msgs::OdometryConstPtr& msg);
		void Visualizer(void);
};

PCStore::PCStore()
{
	sub_pc = nh.subscribe("/velodyne_points", 1, &PCStore::CallbackPC, this);
	sub_odom = nh.subscribe("/combined_odometry", 1, &PCStore::CallbackOdom, this);
	viewer.setBackgroundColor(1, 1, 1);
	viewer.addCoordinateSystem(0.5, "axis");
}

void PCStore::CallbackPC(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	// std::cout << "CALLBACK PC" << std::endl;
	pcl::fromROSMsg(*msg, *cloud_now);
	pc_was_added = false;
}

void PCStore::CallbackOdom(const nav_msgs::OdometryConstPtr& msg)
{
	// std::cout << "CALLBACK ODOM" << std::endl;
	odom_now = *msg;
	time_odom_now = ros::Time::now();
	double dt = (time_odom_now - time_odom_last).toSec();
	time_odom_last = time_odom_now;
	
	if(first_callback_odom){
		odom_last = odom_now;
		dt = 0.0;
	}
	else if(!pc_was_added){
		tf::Quaternion pose_now;
		tf::Quaternion pose_last;
		quaternionMsgToTF(odom_now.pose.pose.orientation, pose_now);
		quaternionMsgToTF(odom_last.pose.pose.orientation, pose_last);
		tf::Quaternion relative_rotation = pose_last*pose_now.inverse();
		relative_rotation.normalize();	
		Eigen::Quaternionf rotation(relative_rotation.w(), relative_rotation.x(), relative_rotation.y(), relative_rotation.z());
		Eigen::Vector3f offset(-odom_last.twist.twist.linear.x*dt, 0.0, 0.0);
		pcl::transformPointCloud(*cloud_stored, *cloud_stored, offset, rotation);
		*cloud_stored  += *cloud_now;
		pc_was_added = true;
		
		odom_last = odom_now;
		
		/*limit storing*/
		if(limit_storing){
			list_num_scanpoints.push_back(cloud_now->points.size());
			if(list_num_scanpoints.size()>limit_num_scans){
				cloud_stored->points.erase(cloud_stored->points.begin(), cloud_stored->points.begin() + list_num_scanpoints[0]);
				list_num_scanpoints.erase(list_num_scanpoints.begin());
			}
			std::cout << "limit storing: true" << std::endl;
			std::cout << "number of stored scans: " << list_num_scanpoints.size() << std::endl;
		}
	}
	Visualizer();

	first_callback_odom = false;
}
void PCStore::Visualizer(void)
{
	viewer.removePointCloud("cloud");
	viewer.addPointCloud(cloud_stored, "cloud");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 0.0, "cloud");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
	
	viewer.spinOnce();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pc_store");
	
	PCStore pc_store;

	ros::spin();
}
