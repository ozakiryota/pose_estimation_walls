#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d.h>
#include <pcl_conversions/pcl_conversions.h>

class PCFittingWalls{
	private:
		ros::NodeHandle nh;
		ros::Subscriber sub;
		/*viewer*/
		pcl::visualization::PCLVisualizer viewer{"pc_normals"};
		/*cloud*/
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud {new pcl::PointCloud<pcl::PointXYZ>};
		pcl::PointCloud<pcl::PointNormal>::Ptr normals {new pcl::PointCloud<pcl::PointNormal>};
		pcl::PointCloud<pcl::PointNormal>::Ptr extracted_normals {new pcl::PointCloud<pcl::PointNormal>};
		pcl::PointCloud<pcl::PointXYZ>::Ptr gaussian_sphere {new pcl::PointCloud<pcl::PointXYZ>};
		pcl::PointCloud<pcl::PointXYZ>::Ptr gaussian_sphere_clustered {new pcl::PointCloud<pcl::PointXYZ>};
		pcl::PointCloud<pcl::PointNormal>::Ptr g_vector {new pcl::PointCloud<pcl::PointNormal>};
		/*kdtree*/
		pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

	public:
		PCFittingWalls();
		void Callback(const sensor_msgs::PointCloud2ConstPtr& msg);
		void GetGVector(void);
		void ClearCloud(void);
		void NormalEstimation(void);
		std::vector<int> KdtreeSearch(pcl::PointXYZ searchpoint, double search_radius);
		double AngleBetweenVectors(pcl::PointNormal v1, pcl::PointNormal v2);
		double ComputeSquareError(Eigen::Vector4f plane_parameters, std::vector<int> indices);
		void PointCluster(void);
		void PointMerge(int index1, int index2, std::vector<int> list_num_belongings);
		void Visualizer(void);
};

PCFittingWalls::PCFittingWalls()
{
	sub = nh.subscribe("/velodyne_points", 1, &PCFittingWalls::Callback, this);
	viewer.setBackgroundColor(1, 1, 1);
	viewer.addCoordinateSystem(0.5, "axis");
	g_vector->points.resize(1);
}

void PCFittingWalls::Callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	std::cout << "CALLBACK" << std::endl;
	GetGVector();
	pcl::fromROSMsg(*msg, *cloud);
	pcl::fromROSMsg(*msg, *normals);
	ClearCloud();
	NormalEstimation();
	PointCluster();
	Visualizer();
}

void PCFittingWalls::GetGVector(void)
{
	/*test*/
	g_vector->points[0].x = 0.0;
	g_vector->points[0].y = 0.0;
	g_vector->points[0].z = 0.0;
	g_vector->points[0].normal_x = 0.0;
	g_vector->points[0].normal_y = 0.0;
	g_vector->points[0].normal_z = -1.0;
}

void PCFittingWalls::ClearCloud(void)
{
	extracted_normals->points.clear();
	gaussian_sphere->points.clear();
}

void PCFittingWalls::NormalEstimation(void)
{
	std::cout << "NORMAL ESTIMATION" << std::endl;
	kdtree.setInputCloud(cloud);
		
	for(size_t i=0;i<normals->points.size();i++){
		/*search neighbor points*/
		std::vector<int> indices;
		float curvature;
		Eigen::Vector4f plane_parameters;
		double search_radius = 0.5;
		indices = KdtreeSearch(cloud->points[i], search_radius);
		/*compute normal*/
		pcl::computePointNormal(*cloud, indices, plane_parameters, curvature);
		
		normals->points[i].normal_x = plane_parameters[0]; 
		normals->points[i].normal_y = plane_parameters[1]; 
		normals->points[i].normal_z = plane_parameters[2]; 
		flipNormalTowardsViewpoint(normals->points[i], 0.0, 0.0, 0.0, normals->points[i].normal_x, normals->points[i].normal_y, normals->points[i].normal_z);

		pcl::PointNormal tmp_normal;
		tmp_normal.x = cloud->points[i].x;
		tmp_normal.y = cloud->points[i].y;
		tmp_normal.z = cloud->points[i].z;
		tmp_normal.normal_x = plane_parameters[0];
		tmp_normal.normal_y = plane_parameters[1];
		tmp_normal.normal_z = plane_parameters[2];
		tmp_normal.curvature = curvature;
		flipNormalTowardsViewpoint(tmp_normal, 0.0, 0.0, 0.0, tmp_normal.normal_x, tmp_normal.normal_y, tmp_normal.normal_z);
		
		/*judge*/		
		const size_t num_neighborpoints = 50;
		if(indices.size()<num_neighborpoints){
			std::cout << ">> indices.size() = " << indices.size() << " < " << num_neighborpoints << ", then skip" << std::endl;
			continue;
		}
		/*judge*/
		// const double threshold_curvature = 1.0e-1;
		// if(curvature>threshold_curvature){
		// 	std::cout << ">> curvature = " << curvature << " > " << threshold_curvature << ", then skip" << std::endl;
		// 	continue;
		// }
		/*judge*/
		const double threshold_angle = 30.0;	//[deg]
		if(fabs(AngleBetweenVectors(tmp_normal, g_vector->points[0])-M_PI/2.0)>threshold_angle/180.0*M_PI){
			std::cout << ">> angle from square angle = " << fabs(AngleBetweenVectors(tmp_normal, g_vector->points[0])-M_PI/2.0) << " > " << threshold_angle << ", then skip" << std::endl;
			continue;
		}
		const double threshold_square_error = 0.0001;
		if(ComputeSquareError(plane_parameters, indices)>threshold_square_error){
			std::cout << ">> square error = " << ComputeSquareError(plane_parameters, indices) << " > " << threshold_square_error << ", then skip" << std::endl;
			continue;
		}

		/*transfer to Gaussian sphere*/
		extracted_normals->points.push_back(tmp_normal);
		pcl::PointXYZ tmp_point;
		tmp_point.x = plane_parameters[0];
		tmp_point.y = plane_parameters[1];
		tmp_point.z = plane_parameters[2];
		gaussian_sphere->points.push_back(tmp_point);
	}
}

std::vector<int> PCFittingWalls::KdtreeSearch(pcl::PointXYZ searchpoint, double search_radius)
{
	std::cout << "KDTREE SEARCH" << std::endl;
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;
	if(kdtree.radiusSearch(searchpoint, search_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance)<=0)	std::cout << "kdtree error" << std::endl;
	return pointIdxRadiusSearch; 
}

double PCFittingWalls::AngleBetweenVectors(pcl::PointNormal v1, pcl::PointNormal v2)
{
	std::cout << "ANGLE BETWEEN VECTORS" << std::endl;
	double dot_product = v1.normal_x*v2.normal_x + v1.normal_y*v2.normal_y + v1.normal_z*v2.normal_z;
	double v1_norm = sqrt(v1.normal_x*v1.normal_x + v1.normal_y*v1.normal_y + v1.normal_z*v1.normal_z);
	double v2_norm = sqrt(v2.normal_x*v2.normal_x + v2.normal_y*v2.normal_y + v2.normal_z*v2.normal_z);
	double angle = acosf(dot_product/(v1_norm*v2_norm));
	return angle;
}

double PCFittingWalls::ComputeSquareError(Eigen::Vector4f plane_parameters, std::vector<int> indices)
{
	std::cout << "COMPUTE SQUARE ERROR" << std::endl;
	double sum_square_error = 0.0;
	for(size_t i=0;i<indices.size();i++){
		double square_error =	(plane_parameters[0]*cloud->points[indices[i]].x
								+plane_parameters[1]*cloud->points[indices[i]].y
								+plane_parameters[2]*cloud->points[indices[i]].z
								+plane_parameters[3])
								*(plane_parameters[0]*cloud->points[indices[i]].x
								+plane_parameters[1]*cloud->points[indices[i]].y
								+plane_parameters[2]*cloud->points[indices[i]].z
								+plane_parameters[3])
								/(plane_parameters[0]*plane_parameters[0]
								+plane_parameters[1]*plane_parameters[1]
								+plane_parameters[2]*plane_parameters[2]);
		sum_square_error += square_error/(double)indices.size();
	}
	return sum_square_error;
}

void PCFittingWalls::PointCluster(void)
{
	std::cout << "POINT CLUSTER" << std::endl;
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_;
	const int k = 2;
	*gaussian_sphere_clustered = *gaussian_sphere;
	std::vector<int> list_num_belongings(gaussian_sphere_clustered->points.size(), 1);
	while(ros::ok()){
		if(gaussian_sphere_clustered->points.size()<k){
			std::cout << ">> finished merging" << std::endl;
			break;
		}

		double shortest_distance;
		int merge_pair_indices[2];
		for(size_t i=0;i<gaussian_sphere_clustered->points.size();i++){
			std::vector<int> pointIdxNKNSearch(k);
			std::vector<float> pointNKNSquaredDistance(k);
			kdtree_.setInputCloud(gaussian_sphere_clustered);
		std::cout << "test1" << std::endl;
			if(kdtree_.nearestKSearch(gaussian_sphere_clustered->points[i], k, pointIdxNKNSearch, pointNKNSquaredDistance)<=0)	std::cout << "kdtree_ error" << std::endl;
		std::cout << "test2" << std::endl;
			if(i==0){
				shortest_distance = pointNKNSquaredDistance[1];
				merge_pair_indices[0] = i;
				merge_pair_indices[1] = pointIdxNKNSearch[1];
			}
			else if(pointNKNSquaredDistance[1]<shortest_distance){
				shortest_distance = pointNKNSquaredDistance[1];
				merge_pair_indices[0] = i;
				merge_pair_indices[1] = pointIdxNKNSearch[1];
			}
		}
		const double threshold_merge_distance = 0.1;
		if(shortest_distance>threshold_merge_distance){
			std::cout << ">> finished merging" << std::endl;
			break;
		}
		else{
			list_num_belongings[merge_pair_indices[0]] += list_num_belongings[merge_pair_indices[1]];
			PointMerge(merge_pair_indices[0], merge_pair_indices[1], list_num_belongings);
			/*erase*/
			gaussian_sphere_clustered->points.erase(gaussian_sphere_clustered->points.begin() + merge_pair_indices[1]);
			list_num_belongings.erase(list_num_belongings.begin() + merge_pair_indices[1]);
		}
	}
}

void PCFittingWalls::PointMerge(int index1, int index2, std::vector<int> list_num_belongings)
{
	std::cout << "POINT MERGE" << std::endl;
}

void PCFittingWalls::Visualizer(void)
{
	std::cout << "VISUALIZER" << std::endl;

	viewer.removePointCloud("cloud");
	viewer.addPointCloud(cloud, "cloud");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 0.0, "cloud");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
	
	viewer.removePointCloud("normals");
	viewer.addPointCloudNormals<pcl::PointNormal>(normals, 1, 0.5, "normals");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "normals");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1, "normals");
	
	viewer.removePointCloud("extracted_normals");
	viewer.addPointCloudNormals<pcl::PointNormal>(extracted_normals, 1, 0.5, "extracted_normals");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 1.0, "extracted_normals");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, "extracted_normals");

	viewer.removePointCloud("gaussian_sphere");
	viewer.addPointCloud(gaussian_sphere, "gaussian_sphere");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "gaussian_sphere");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "gaussian_sphere");

	viewer.spinOnce();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pc_fitting_walls");
	std::cout << "PC FITTING WALLS" << std::endl;
	
	PCFittingWalls pc_fitting_walls;
	ros::spin();
}
