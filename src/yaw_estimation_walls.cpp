#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/cloud_viewer.h>
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
		pcl::PointCloud<pcl::PointXYZ>::Ptr walls_now {new pcl::PointCloud<pcl::PointXYZ>};
		pcl::PointCloud<pcl::PointXYZ>::Ptr walls_now_rotated {new pcl::PointCloud<pcl::PointXYZ>};
		pcl::PointCloud<pcl::PointXYZ>::Ptr walls_last {new pcl::PointCloud<pcl::PointXYZ>};
		// std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> list_walls;
		/*poses*/
		tf::Quaternion pose_now;
		tf::Quaternion pose_last;
		/*flags*/
		bool first_callback_pose = true;
		/*viewer*/
		pcl::visualization::PCLVisualizer viewer{"pc_walls"};
	public:
		YawEstimationWalls();
		void CallbackPose(const geometry_msgs::PoseStampedConstPtr &msg);
		void CallbackNormals(const sensor_msgs::PointCloud2ConstPtr &msg);
		void MatchWalls(void);
		void Visualizer(void);
};

YawEstimationWalls::YawEstimationWalls()
{
	sub_walls = nh.subscribe("/g_and_walls", 1, &YawEstimationWalls::CallbackNormals, this);
	sub_pose = nh.subscribe("/pose_ekf", 1, &YawEstimationWalls::CallbackPose, this);
	pub = nh.advertise<std_msgs::Float64>("/yaw_rate_walls", 1);
	viewer.setBackgroundColor(1, 1, 1);
	viewer.addCoordinateSystem(0.2, "axis");
}

void YawEstimationWalls::CallbackPose(const geometry_msgs::PoseStampedConstPtr &msg)
{
	std::cout << "CALLBACK POSE" << std::endl;
	quaternionMsgToTF(msg->pose.orientation, pose_now);
	if(first_callback_pose)	pose_last = pose_now;
	
	first_callback_pose = false;
}

void YawEstimationWalls::CallbackNormals(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	std::cout << "CALLBACK NORMALS" << std::endl;
	pcl::PointCloud<pcl::PointNormal>::Ptr tmp_normals (new pcl::PointCloud<pcl::PointNormal>);
	pcl::fromROSMsg(*msg, *tmp_normals);
	walls_now->points.clear();
	/*convert point type*/
	for(size_t i=1;i<tmp_normals->points.size();i++){
		pcl::PointXYZ tmp_point;
		tmp_point.x = tmp_normals->points[i].normal_x;
		tmp_point.y = tmp_normals->points[i].normal_y;
		tmp_point.z = tmp_normals->points[i].normal_z;
		walls_now->points.push_back(tmp_point);
	}

	if(!first_callback_pose)	MatchWalls();
	pose_last = pose_now;

	Visualizer();
}

void YawEstimationWalls::MatchWalls(void)
{
	std::cout << "REGISTER WALLS" << std::endl;
	if(walls_last->points.empty()){
		walls_last = walls_now;
	}
	else{
		/*rotate wall points*/
		tf::Quaternion relative_rotation = pose_now*pose_last.inverse();
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
		const double threshold_matching_distance = 0.1;
		// std::vector<bool> list_matched(walls_last->points.size(), false);
		std::vector<double> list_yawrate;
		for(size_t i=0;i<walls_last->points.size();i++){
			if(kdtree.nearestKSearch(walls_last->points[i], k, pointIdxNKNSearch, pointNKNSquaredDistance)<=0)	std::cout << "kdtree error" << std::endl;
			if(pointNKNSquaredDistance[0]<threshold_matching_distance){
				double yawrate;
				list_yawrate.push_back(yawrate);
			}
		}
		walls_last = walls_now;
	}
}

void YawEstimationWalls::Visualizer(void)
{
	std::cout << "VISUALIZER" << std::endl;
	
	viewer.removePointCloud("walls_now");
	viewer.addPointCloud(walls_now, "walls_now");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "walls_now");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "walls_now");
	
	viewer.removePointCloud("walls_now_rotated");
	viewer.addPointCloud(walls_now_rotated, "walls_now_rotated");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "walls_now_rotated");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "walls_now_rotated");

	viewer.removePointCloud("walls_last");
	viewer.addPointCloud(walls_last, "walls_last");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "walls_last");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "walls_last");

	viewer.spinOnce();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "yaw_estimation_walls");
	
	YawEstimationWalls yaw_estimation_walls;

	ros::spin();
}
