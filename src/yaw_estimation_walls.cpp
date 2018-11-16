#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <tf/tf.h>

class YawEstimationWalls{
	private:
		ros::NodeHandle nh;
		/*subscribe*/
		ros::Subscriber sub_pc;
		ros::Subscriber sub_odom;
		/*publish*/
		ros::Publisher pub_pose;
		/*struct*/
		struct WallInfo{
			pcl::PointXYZ point;
			nav_msgs::Odometry odom;
			Eigen::MatrixXd X;
			Eigen::MatrixXd P;
			int count_nomatch;
		};
		/*pcl*/
		pcl::visualization::PCLVisualizer viewer{"d-gaussian sphere"};
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud {new pcl::PointCloud<pcl::PointXYZ>};
		pcl::PointCloud<pcl::PointNormal>::Ptr normals {new pcl::PointCloud<pcl::PointNormal>};
		pcl::PointCloud<pcl::PointXYZ>::Ptr d_gauss {new pcl::PointCloud<pcl::PointXYZ>};
		pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
		/*objects*/
		std::vector<WallInfo> list_walls;
		nav_msgs::Odometry odom_now;
		/*flags*/
		bool first_callback_odom = true;
	public:
		YawEstimationWalls();
		void CallbackPC(const sensor_msgs::PointCloud2ConstPtr &msg);
		void CallbackOdom(const nav_msgs::OdometryConstPtr& msg);
		void ClearCloud(void);
		void NormalEstimation(void);
		std::vector<int> KdtreeSearch(pcl::PointXYZ searchpoint, double search_radius);
		double ComputeSquareError(Eigen::Vector4f plane_parameters, std::vector<int> indices);
		void PointCluster(void);
		void Visualization(void);
};

YawEstimationWalls::YawEstimationWalls()
{
	sub_pc = nh.subscribe("/velodyne_points", 1, &YawEstimationWalls::CallbackPC, this);
	sub_odom = nh.subscribe("/gyrodometry", 1, &YawEstimationWalls::CallbackOdom, this);
	pub_pose = nh.advertise<geometry_msgs::PoseStamped>("/pose", 1);
	viewer.setBackgroundColor(1, 1, 1);
	viewer.addCoordinateSystem(0.5, "axis");
}

void YawEstimationWalls::CallbackPC(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	pcl::fromROSMsg(*msg, *cloud);
	ClearCloud();
	NormalEstimation();
	Visualization();
}

void YawEstimationWalls::CallbackOdom(const nav_msgs::OdometryConstPtr& msg)
{
	odom_now = *msg;

	first_callback_odom = false;
}

void YawEstimationWalls::ClearCloud(void)
{
	d_gauss->points.clear();
	normals->points.clear();
}

void YawEstimationWalls::NormalEstimation(void)
{
	std::cout << "NORMAL ESTIMATION" << std::endl;
	kdtree.setInputCloud(cloud);

	const size_t skip_step = 5;
	for(size_t i=0;i<cloud->points.size();i+=skip_step){
		/*search neighbor points*/
		std::vector<int> indices;
		float curvature;
		Eigen::Vector4f plane_parameters;
		double laser_distance = sqrt(cloud->points[i].x*cloud->points[i].x + cloud->points[i].y*cloud->points[i].y + cloud->points[i].z*cloud->points[i].z);
		const double search_radius_min = 0.5;
		const double ratio = 0.09;
		double search_radius = ratio*laser_distance;
		if(search_radius<search_radius_min)	search_radius = search_radius_min;
		indices = KdtreeSearch(cloud->points[i], search_radius);
		
		/*judge*/		
		const size_t num_neighborpoints = 50;
		if(indices.size()<num_neighborpoints){
			std::cout << ">> indices.size() = " << indices.size() << " < " << num_neighborpoints << ", then skip" << std::endl;
			continue;
		}
		/*compute normal*/
		pcl::computePointNormal(*cloud, indices, plane_parameters, curvature);
		/*judge*/
		const double threshold_square_error = 0.01;
		if(ComputeSquareError(plane_parameters, indices)>threshold_square_error){
			std::cout << ">> square error = " << ComputeSquareError(plane_parameters, indices) << " > " << threshold_square_error << ", then skip" << std::endl;
			continue;
		}
		/*delete nan*/
		if(std::isnan(plane_parameters[0]) || std::isnan(plane_parameters[1]) || std::isnan(plane_parameters[2])){
			std::cout << ">> this point has NAN, then skip" << std::endl;
			continue;
		}
		/*input*/
		std::cout << ">> ok, then input" << std::endl;
		pcl::PointXYZ tmp_point;
		tmp_point.x = -plane_parameters[3]*plane_parameters[0];
		tmp_point.y = -plane_parameters[3]*plane_parameters[1];
		tmp_point.z = -plane_parameters[3]*plane_parameters[2];
		d_gauss->points.push_back(tmp_point);
		/*input(just for visualization)*/
		pcl::PointNormal tmp_normal;
		tmp_normal.x = cloud->points[i].x;
		tmp_normal.y = cloud->points[i].y;
		tmp_normal.z = cloud->points[i].z;
		tmp_normal.normal_x = plane_parameters[0];
		tmp_normal.normal_y = plane_parameters[1];
		tmp_normal.normal_z = plane_parameters[2];
		tmp_normal.curvature = curvature;
		flipNormalTowardsViewpoint(tmp_normal, 0.0, 0.0, 0.0, tmp_normal.normal_x, tmp_normal.normal_y, tmp_normal.normal_z);
		normals->points.push_back(tmp_normal);
	}
}

std::vector<int> YawEstimationWalls::KdtreeSearch(pcl::PointXYZ searchpoint, double search_radius)
{
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;
	if(kdtree.radiusSearch(searchpoint, search_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance)<=0)	std::cout << "kdtree error" << std::endl;
	return pointIdxRadiusSearch; 
}

double YawEstimationWalls::ComputeSquareError(Eigen::Vector4f plane_parameters, std::vector<int> indices)
{
	double sum_square_error = 0.0;
	for(size_t i=0;i<indices.size();i++){
		double square_error =	fabs(plane_parameters[0]*cloud->points[indices[i]].x
									+plane_parameters[1]*cloud->points[indices[i]].y
									+plane_parameters[2]*cloud->points[indices[i]].z
									+plane_parameters[3])
								/sqrt(plane_parameters[0]*plane_parameters[0]
									+plane_parameters[1]*plane_parameters[1]
									+plane_parameters[2]*plane_parameters[2]);
		sum_square_error += square_error/(double)indices.size();
	}
	return sum_square_error;
}

void YawEstimationWalls::PointCluster(void)
{
}

void YawEstimationWalls::Visualization(void)
{
	viewer.removeAllPointClouds();

	viewer.addPointCloud(cloud, "cloud");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 0.0, "cloud");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
	
	viewer.addPointCloud(d_gauss, "d_gauss");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "d_gauss");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "d_gauss");
	
	viewer.addPointCloudNormals<pcl::PointNormal>(normals, 1, 0.5, "normals");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 1.0, "normals");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1, "normals");
	
	viewer.spinOnce();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "yaw_estimation_walls");
	
	YawEstimationWalls yaw_estimation_walls;

	ros::spin();
}
