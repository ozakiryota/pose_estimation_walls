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
	// if(!first_callback_odom){
	// 	tf::Quaternion pose_now;
	// 	tf::Quaternion pose_last;
	// 	quaternionMsgToTF(odom_now.pose.pose.orientation, pose_now);
	// 	quaternionMsgToTF(odom_last.pose.pose.orientation, pose_last);
	// 	tf::Quaternion relative_rotation = pose_last*pose_now.inverse();
	// 	Eigen::Quaternionf rotation(relative_rotation.w(), relative_rotation.x(), relative_rotation.y(), relative_rotation.z());
	// 	Eigen::Vector3f offset(
	// 			odom_last.pose.pose.position.x - odom_now.pose.pose.position.x,
	// 			odom_last.pose.pose.position.y - odom_now.pose.pose.position.y,
	// 			odom_last.pose.pose.position.z - odom_now.pose.pose.position.z);
	// 	pcl::transformPointCloud(*cloud, *cloud, offset, rotation);
    //
	// 	pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	// 	pcl::fromROSMsg(*msg, *tmp_cloud);
	// 	*cloud += *tmp_cloud;
    //
	// 	odom_last = odom_now;
	// }
	pcl::fromROSMsg(*msg, *cloud_now);
}

void PCStore::CallbackOdom(const nav_msgs::OdometryConstPtr& msg)
{
	odom_now = *msg;
	if(first_callback_odom)	odom_last = odom_now;
	else{
		tf::Quaternion pose_now;
		tf::Quaternion pose_last;
		quaternionMsgToTF(odom_now.pose.pose.orientation, pose_now);
		quaternionMsgToTF(odom_last.pose.pose.orientation, pose_last);
		tf::Quaternion relative_rotation = pose_last*pose_now.inverse();
		Eigen::Quaternionf rotation(relative_rotation.w(), relative_rotation.x(), relative_rotation.y(), relative_rotation.z());
		Eigen::Vector3f offset(
				odom_last.pose.pose.position.x - odom_now.pose.pose.position.x,
				odom_last.pose.pose.position.y - odom_now.pose.pose.position.y,
				odom_last.pose.pose.position.z - odom_now.pose.pose.position.z);
		pcl::transformPointCloud(*cloud_stored, *cloud_stored, offset, rotation);
		*cloud_stored  += *cloud_now;
			
		odom_last = odom_now;
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
