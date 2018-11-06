#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

class PlaneSegmentation{
	private:
		ros::NodeHandle nh;
		ros::Subscriber sub;
		pcl::visualization::PCLVisualizer viewer{"plane segmentation"};
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud {new pcl::PointCloud<pcl::PointXYZ>};
		std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> planes;
	public:
		PlaneSegmentation();
		void Callback(const sensor_msgs::PointCloud2ConstPtr& msg);
		void Extraction(void);
		void Visualization(void);
};

PlaneSegmentation::PlaneSegmentation()
{
	sub = nh.subscribe("/velodyne_points", 1, &PlaneSegmentation::Callback, this);
	viewer.setBackgroundColor(1, 1, 1);
	viewer.addCoordinateSystem(0.5, "axis");
}

void PlaneSegmentation::Callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
	pcl::fromROSMsg(*msg, *cloud);
	planes.clear();
	Extraction();
	Visualization();
}

void PlaneSegmentation::Extraction(void)
{
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	// seg.setMethodType (pcl::SAC_LMEDS);
	// seg.setMethodType (pcl::SAC_MSAC);
	seg.setDistanceThreshold (0.05);

	pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud {new pcl::PointCloud<pcl::PointXYZ>};
	*tmp_cloud = *cloud;
	while(ros::ok()){
		if(tmp_cloud->points.empty())	break;
		if(tmp_cloud->points.size()<cloud->points.size()*0.2)	break;
		seg.setInputCloud(tmp_cloud);
		seg.segment(*inliers, *coefficients);
		if(inliers->indices.empty())	break;

		pcl::ExtractIndices<pcl::PointXYZ> ec;
		ec.setInputCloud(tmp_cloud);  
		ec.setIndices(inliers);
		pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_plane {new pcl::PointCloud<pcl::PointXYZ>};
		ec.setNegative(false);
		ec.filter(*tmp_plane);
		if(true)	planes.push_back(tmp_plane);
		else{
			if(coefficients->values[2]<0.8)	planes.push_back(tmp_plane);
		}
		ec.setNegative(true);
		ec.filter(*tmp_cloud);
	}
	std::cout << "planes.size() = " << planes.size() << std::endl;
}

void PlaneSegmentation::Visualization(void)
{
	viewer.removeAllPointClouds();
	
	viewer.addPointCloud(cloud, "cloud");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 0.0, "cloud");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");

	double color_ratio = 7.0/(double)planes.size();
	int step = 1;
	std::vector<int> rgb(3, 0);
	for(size_t i=0;i<planes.size();i++){
		std::string name = "plane" + std::to_string(i);
		rgb[0] += step;
		for(size_t i=0;i<rgb.size()-1;i++){
			if(rgb[i]>step){
				rgb[i] = 0;
				rgb[i+1] += step;
			}
		}
		viewer.addPointCloud(planes[i], name);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, rgb[0]*color_ratio, rgb[1]*color_ratio, rgb[2]*color_ratio, name);
		viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, name);
		if(rgb[0]==step && rgb[1]==step && rgb[2]==step){
			step++;
			rgb = {0, 0, 0};
		}
	}
	
	viewer.spinOnce();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "multi_plane_segmentation");
	
	PlaneSegmentation plane_segmentation;

	ros::spin();
}
