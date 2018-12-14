#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>

class PCFittingWalls{
	private:
		ros::NodeHandle nh;
		/*subscribe*/
		ros::Subscriber sub_pc;
		ros::Subscriber sub_pose;
		/*publish*/
		ros::Publisher pub;
		/*viewer*/
		pcl::visualization::PCLVisualizer viewer{"Gaussian Sphere"};
		/*cloud*/
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud {new pcl::PointCloud<pcl::PointXYZ>};
		pcl::PointCloud<pcl::PointNormal>::Ptr normals {new pcl::PointCloud<pcl::PointNormal>};
		pcl::PointCloud<pcl::PointNormal>::Ptr extracted_normals {new pcl::PointCloud<pcl::PointNormal>};
		pcl::PointCloud<pcl::InterestPoint>::Ptr gaussian_sphere {new pcl::PointCloud<pcl::InterestPoint>};
		pcl::PointCloud<pcl::InterestPoint>::Ptr gaussian_sphere_clustered {new pcl::PointCloud<pcl::InterestPoint>};
		pcl::PointCloud<pcl::PointXYZ>::Ptr gaussian_sphere_clustered_weighted {new pcl::PointCloud<pcl::PointXYZ>};
		pcl::PointCloud<pcl::PointNormal>::Ptr gaussian_sphere_clustered_n {new pcl::PointCloud<pcl::PointNormal>};	//just for vizualizing
		pcl::PointCloud<pcl::PointNormal>::Ptr g_vector_from_ekf {new pcl::PointCloud<pcl::PointNormal>};
		pcl::PointCloud<pcl::PointNormal>::Ptr g_vector {new pcl::PointCloud<pcl::PointNormal>};
		/*kdtree*/
		pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_xyz;
		pcl::KdTreeFLANN<pcl::InterestPoint> kdtree_xyzs;
		/*flags*/
		bool g_estimation_success;

	public:
		PCFittingWalls();
		void CallbackPC(const sensor_msgs::PointCloud2ConstPtr& msg);
		void CallbackPose(const geometry_msgs::PoseStampedConstPtr &msg);
		Eigen::MatrixXd FrameRotation(geometry_msgs::Quaternion q, Eigen::MatrixXd X, bool from_global_to_local);
		void ClearCloud(void);
		void NormalEstimation(void);
		std::vector<int> KdtreeSearch(pcl::PointXYZ searchpoint, double search_radius);
		double AngleBetweenVectors(pcl::PointNormal v1, pcl::PointNormal v2);
		double ComputeSquareError(Eigen::Vector4f plane_parameters, std::vector<int> indices);
		void PointCluster(void);
		void PointPreCluster(void);
		pcl::InterestPoint PointMerge(int index1, int index2);
		bool GVectorEstimation(void);
		void PartialRotation(void);
		void Visualizer(void);
		void Publisher(void);
};

PCFittingWalls::PCFittingWalls()
{
	sub_pc = nh.subscribe("/velodyne_points", 1, &PCFittingWalls::CallbackPC, this);
	sub_pose = nh.subscribe("/pose_ekf", 1, &PCFittingWalls::CallbackPose, this);
	pub = nh.advertise<sensor_msgs::PointCloud2>("/g_and_walls", 1);
	viewer.setBackgroundColor(1, 1, 1);
	viewer.addCoordinateSystem(0.5, "axis");
	viewer.setCameraPosition(-5.0, -10.0, 6.0, 0.0, 0.0, 1.0);
	g_vector_from_ekf->points.resize(1);
	g_vector_from_ekf->points[0].x = 0.0;
	g_vector_from_ekf->points[0].y = 0.0;
	g_vector_from_ekf->points[0].z = 0.0;
	g_vector_from_ekf->points[0].normal_x = 0.0;
	g_vector_from_ekf->points[0].normal_y = 0.0;
	g_vector_from_ekf->points[0].normal_z = -1.0;
	g_vector->points.resize(1);
	g_vector->points[0].x = 0.0;
	g_vector->points[0].y = 0.0;
	g_vector->points[0].z = 0.0;
}

void PCFittingWalls::CallbackPC(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	std::cout << "CALLBACK PC" << std::endl;
	pcl::fromROSMsg(*msg, *cloud);
	ClearCloud();
	NormalEstimation();
	PointCluster();
	g_estimation_success = GVectorEstimation();
	Visualizer();
	Publisher();
}

void PCFittingWalls::CallbackPose(const geometry_msgs::PoseStampedConstPtr &msg)
{
	std::cout << "CALLBACK POSE" << std::endl;
	Eigen::MatrixXd G_global(3, 1);
	G_global <<	0.0,
			 	0.0,
				-1.0;
	Eigen::MatrixXd G_local(3, 1);
	G_local = FrameRotation(msg->pose.orientation, G_global, true);
	
	g_vector_from_ekf->points[0].normal_x = G_local(0, 0);
	g_vector_from_ekf->points[0].normal_y = G_local(1, 0);
	g_vector_from_ekf->points[0].normal_z = G_local(2, 0);
}

Eigen::MatrixXd PCFittingWalls::FrameRotation(geometry_msgs::Quaternion q, Eigen::MatrixXd X, bool from_global_to_local)
{
	if(!from_global_to_local)    q.w *= -1;
	Eigen::MatrixXd Rot(3, 3); 
	Rot <<	q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z,  2*(q.x*q.y + q.w*q.z),	2*(q.x*q.z - q.w*q.y),
			2*(q.x*q.y - q.w*q.z),	q.w*q.w - q.x*q.x + q.y*q.y - q.z*q.z,	2*(q.y*q.z + q.w*q.x),
			2*(q.x*q.z + q.w*q.y),	2*(q.y*q.z - q.w*q.x),	q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z;
	// std::cout << "Rot*X = " << std::endl << Rot*X << std::endl;
	if(from_global_to_local)    return Rot*X;
	else    return Rot.inverse()*X;
}

void PCFittingWalls::ClearCloud(void)
{
	normals->points.clear();
	extracted_normals->points.clear();
	gaussian_sphere->points.clear();
	gaussian_sphere_clustered_weighted->points.clear();
	gaussian_sphere_clustered_n->points.clear();
}

void PCFittingWalls::NormalEstimation(void)
{
	std::cout << "NORMAL ESTIMATION" << std::endl;
	kdtree_xyz.setInputCloud(cloud);

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
		/*compute normal*/
		pcl::computePointNormal(*cloud, indices, plane_parameters, curvature);

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
		if(fabs(AngleBetweenVectors(tmp_normal, g_vector_from_ekf->points[0])-M_PI/2.0)>threshold_angle/180.0*M_PI){
			std::cout << ">> angle from square angle = " << fabs(AngleBetweenVectors(tmp_normal, g_vector_from_ekf->points[0])-M_PI/2.0) << " > " << threshold_angle << ", then skip" << std::endl;
			continue;
		}
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
		extracted_normals->points.push_back(tmp_normal);
		pcl::InterestPoint tmp_point;
		tmp_point.x = plane_parameters[0];
		tmp_point.y = plane_parameters[1];
		tmp_point.z = plane_parameters[2];
		tmp_point.strength = 1;
		gaussian_sphere->points.push_back(tmp_point);
	}
}

std::vector<int> PCFittingWalls::KdtreeSearch(pcl::PointXYZ searchpoint, double search_radius)
{
	std::cout << "KDTREE SEARCH" << std::endl;
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;
	if(kdtree_xyz.radiusSearch(searchpoint, search_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance)<=0)	std::cout << "kdtree error" << std::endl;
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

void PCFittingWalls::PointCluster(void)
{
	std::cout << "POINT CLUSTER" << std::endl;
	const int k = 2;
	pcl::copyPointCloud(*gaussian_sphere, *gaussian_sphere_clustered);
	
	/*pre cluster*/
	std::cout << "gaussian_sphere_clustered->points.size() = " << gaussian_sphere_clustered->points.size() << std::endl;
	const int threshold_num_points_before_hierarchical_cluster = 250;
	int num_times_precluster = gaussian_sphere_clustered->points.size()/threshold_num_points_before_hierarchical_cluster;
	for(int i=0;i<num_times_precluster;i++){
		if(gaussian_sphere_clustered->points.size()>threshold_num_points_before_hierarchical_cluster)	PointPreCluster();
		else	break;
	}
	
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
			kdtree_xyzs.setInputCloud(gaussian_sphere_clustered);
			if(kdtree_xyzs.nearestKSearch(gaussian_sphere_clustered->points[i], k, pointIdxNKNSearch, pointNKNSquaredDistance)<=0)	std::cout << "kdtree error" << std::endl;
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
		if(sqrt(shortest_distance)>threshold_merge_distance){
			std::cout << ">> finished merging" << std::endl;
			break;
		}
		else{
			/*merge*/
			gaussian_sphere_clustered->points[merge_pair_indices[0]] = PointMerge(merge_pair_indices[0], merge_pair_indices[1]);
			/*erase*/
			gaussian_sphere_clustered->points.erase(gaussian_sphere_clustered->points.begin() + merge_pair_indices[1]);
		}
	}
	/*erase outlier*/
	const int threshold_num_belongings = 30;
	for(size_t i=0;i<gaussian_sphere_clustered->points.size();i++){
		if(gaussian_sphere_clustered->points[i].strength<threshold_num_belongings){
			gaussian_sphere_clustered->points.erase(gaussian_sphere_clustered->points.begin() + i);
			i--;
		}
		else{
			/*normalization*/
			double norm = sqrt(gaussian_sphere_clustered->points[i].x*gaussian_sphere_clustered->points[i].x + gaussian_sphere_clustered->points[i].y*gaussian_sphere_clustered->points[i].y + gaussian_sphere_clustered->points[i].z*gaussian_sphere_clustered->points[i].z);
			gaussian_sphere_clustered->points[i].x /= norm;
			gaussian_sphere_clustered->points[i].y /= norm;
			gaussian_sphere_clustered->points[i].z /= norm;
			/*input*/
			pcl::PointNormal tmp_normal;
			tmp_normal.x = 0.0;
			tmp_normal.y = 0.0;
			tmp_normal.z = 0.0;
			tmp_normal.normal_x = gaussian_sphere_clustered->points[i].x;
			tmp_normal.normal_y = gaussian_sphere_clustered->points[i].y;
			tmp_normal.normal_z = gaussian_sphere_clustered->points[i].z;
			gaussian_sphere_clustered_n->points.push_back(tmp_normal);
		}
	}
	/*give weight*/
	for(size_t i=0;i<gaussian_sphere_clustered->points.size();i++){
		pcl::PointXYZ tmp_point;
		tmp_point.x = gaussian_sphere_clustered->points[i].strength*gaussian_sphere_clustered->points[i].x;
		tmp_point.y = gaussian_sphere_clustered->points[i].strength*gaussian_sphere_clustered->points[i].y;
		tmp_point.z = gaussian_sphere_clustered->points[i].strength*gaussian_sphere_clustered->points[i].z;
		gaussian_sphere_clustered_weighted->points.push_back(tmp_point);
	}
}

void PCFittingWalls::PointPreCluster(void)
{
	std::cout << "POINT PRE CLUSTER" << std::endl;
	const int k = 2;
	kdtree_xyzs.setInputCloud(gaussian_sphere_clustered);
	const double threshold_merge_distance = 0.05;
	std::vector<bool> list_point_is_merged(gaussian_sphere_clustered->points.size(), false);
	pcl::PointIndices::Ptr indices_to_be_erased (new pcl::PointIndices);
	for(size_t i=0;i<gaussian_sphere_clustered->points.size();i++){
		if(list_point_is_merged[i]) continue;
		std::vector<int> pointIdxNKNSearch(k);
		std::vector<float> pointNKNSquaredDistance(k);
		if(kdtree_xyzs.nearestKSearch(gaussian_sphere_clustered->points[i], k, pointIdxNKNSearch, pointNKNSquaredDistance)<=0)	std::cout << "kdtree error" << std::endl;
		if(!list_point_is_merged[pointIdxNKNSearch[1]] && sqrt(pointNKNSquaredDistance[1])<threshold_merge_distance){
			/*merge*/
			gaussian_sphere_clustered->points[i] = PointMerge(i, pointIdxNKNSearch[1]);
			list_point_is_merged[i] = true;
			list_point_is_merged[pointIdxNKNSearch[1]] = true;
			/*prepare for erasing*/
			indices_to_be_erased->indices.push_back(pointIdxNKNSearch[1]);
		}
	}
	/*erase*/
	pcl::ExtractIndices<pcl::InterestPoint> ec;
	ec.setInputCloud(gaussian_sphere_clustered);
	ec.setIndices(indices_to_be_erased);
	ec.setNegative(true);
	ec.filter(*gaussian_sphere_clustered);
	std::cout << "indices_to_be_erased->indices.size() = " << indices_to_be_erased->indices.size() << std::endl;
}

pcl::InterestPoint PCFittingWalls::PointMerge(int index1, int index2)
{
	std::cout << "POINT MERGE" << std::endl;
	pcl::InterestPoint p;
	pcl::InterestPoint p1 = gaussian_sphere_clustered->points[index1];
	pcl::InterestPoint p2 = gaussian_sphere_clustered->points[index2];
	p.x = (p1.strength*p1.x + p2.strength*p2.x)/(p1.strength + p2.strength);
	p.y = (p1.strength*p1.y + p2.strength*p2.y)/(p1.strength + p2.strength);
	p.z = (p1.strength*p1.z + p2.strength*p2.z)/(p1.strength + p2.strength);
	p.strength = p1.strength + p2.strength;	//number of points that belong
	return p;
}

bool PCFittingWalls::GVectorEstimation(void)
{
	std::cout << "G VECTOR ESTIMATION" << std::endl;
	if(gaussian_sphere_clustered->points.size()==0){
		std::cout << ">> 0 normal" << std::endl;
		return false;
	}
	else if(gaussian_sphere_clustered->points.size()==1){
		std::cout << ">> 1 normal" << std::endl;
		PartialRotation();
	}
	else if(gaussian_sphere_clustered->points.size()==2){
		std::cout << ">> 2 normals" << std::endl;
		/*cross product*/
		g_vector->points[0].normal_x = gaussian_sphere_clustered->points[0].y*gaussian_sphere_clustered->points[1].z - gaussian_sphere_clustered->points[0].z*gaussian_sphere_clustered->points[1].y;
		g_vector->points[0].normal_y = gaussian_sphere_clustered->points[0].z*gaussian_sphere_clustered->points[1].x - gaussian_sphere_clustered->points[0].x*gaussian_sphere_clustered->points[1].z;
		g_vector->points[0].normal_z = gaussian_sphere_clustered->points[0].x*gaussian_sphere_clustered->points[1].y - gaussian_sphere_clustered->points[0].y*gaussian_sphere_clustered->points[1].x;
	}
	else if(gaussian_sphere_clustered->points.size()>2){
		std::cout << ">> more than 3 normals" << std::endl;
		Eigen::Vector4f plane_parameters;
		float curvature;
		pcl::computePointNormal(*gaussian_sphere_clustered_weighted, plane_parameters, curvature);
		g_vector->points[0].normal_x = plane_parameters[0];
		g_vector->points[0].normal_y = plane_parameters[1];
		g_vector->points[0].normal_z = plane_parameters[2];
	}
	/*flip*/
	flipNormalTowardsViewpoint(g_vector->points[0], 0.0, 0.0, -100.0, g_vector->points[0].normal_x, g_vector->points[0].normal_y, g_vector->points[0].normal_z);
	/*if angle variation is too large, estimation would be wrong*/
	const double threshold_angle_variation = 60.0;	//[deg]
	if(fabs(AngleBetweenVectors(g_vector->points[0], g_vector_from_ekf->points[0]))>threshold_angle_variation/180.0*M_PI){
		std::cout << ">> angle variation of g in 1 step is too large and would be wrong " << std::endl;
		return false;
	}
	/*normalization*/
	double norm_g = sqrt(g_vector->points[0].normal_x*g_vector->points[0].normal_x + g_vector->points[0].normal_y*g_vector->points[0].normal_y + g_vector->points[0].normal_z*g_vector->points[0].normal_z);
	g_vector->points[0].normal_x /= norm_g;
	g_vector->points[0].normal_y /= norm_g;
	g_vector->points[0].normal_z /= norm_g;
	return true;
}

void PCFittingWalls::PartialRotation(void)
{
	/*projection*/
	double dot_product =
		g_vector_from_ekf->points[0].normal_x*gaussian_sphere_clustered->points[0].x
		+ g_vector_from_ekf->points[0].normal_y*gaussian_sphere_clustered->points[0].y
		+ g_vector_from_ekf->points[0].normal_z*gaussian_sphere_clustered->points[0].z;
	g_vector->points[0].normal_x = g_vector_from_ekf->points[0].normal_x - dot_product*gaussian_sphere_clustered->points[0].x;
	g_vector->points[0].normal_y = g_vector_from_ekf->points[0].normal_y - dot_product*gaussian_sphere_clustered->points[0].y;
	g_vector->points[0].normal_z = g_vector_from_ekf->points[0].normal_z - dot_product*gaussian_sphere_clustered->points[0].z;
}

void PCFittingWalls::Visualizer(void)
{
	std::cout << "VISUALIZER" << std::endl;
	
	viewer.removeAllPointClouds();

	viewer.addPointCloud(cloud, "cloud");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 0.0, "cloud");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
	
	viewer.addPointCloudNormals<pcl::PointNormal>(normals, 1, 0.5, "normals");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "normals");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1, "normals");
	
	viewer.addPointCloudNormals<pcl::PointNormal>(extracted_normals, 1, 0.5, "extracted_normals");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 1.0, "extracted_normals");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, "extracted_normals");

	viewer.addPointCloud<pcl::InterestPoint>(gaussian_sphere, "gaussian_sphere");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "gaussian_sphere");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "gaussian_sphere");

	viewer.addPointCloudNormals<pcl::PointNormal>(gaussian_sphere_clustered_n, 1, 1.0, "gaussian_sphere_clustered_n");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.8, 0.0, "gaussian_sphere_clustered_n");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, "gaussian_sphere_clustered_n");
	
	viewer.addPointCloudNormals<pcl::PointNormal>(g_vector, 1, 1.0, "g_vector");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "g_vector");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, "g_vector");

	viewer.spinOnce();
}

void PCFittingWalls::Publisher()
{
	std::cout << "PUBLISHER" << std::endl;
	// if(g_estimation_success){
	// 	pcl::PointCloud<pcl::PointNormal>::Ptr g_and_walls (new pcl::PointCloud<pcl::PointNormal>);
	// 	g_and_walls->header = cloud->header;
	// 	g_and_walls->points.push_back(g_vector->points[0]);
	// 	for(size_t i=0;i<gaussian_sphere_clustered_n->points.size();i++){
	// 		g_and_walls->points.push_back(gaussian_sphere_clustered_n->points[i]);	//points[0]:gravity, points[1~]:wall_normals
	// 	}
	// 	sensor_msgs::PointCloud2 ros_g_and_normals;
	// 	pcl::toROSMsg(*g_and_walls, ros_g_and_normals);
	// 	pub.publish(ros_g_and_normals);
	// }
	if(g_estimation_success){
		pcl::PointCloud<pcl::InterestPoint>::Ptr g_and_walls (new pcl::PointCloud<pcl::InterestPoint>);	//points[0]:gravity, points[1~]:wall_normals
		g_and_walls->header = cloud->header;
		pcl::InterestPoint g;
		g.x = g_vector->points[0].normal_x;
		g.y = g_vector->points[0].normal_y;
		g.z = g_vector->points[0].normal_z;
		g.strength = 0;
		g_and_walls->points.push_back(g);
		for(size_t i=0;i<gaussian_sphere_clustered->points.size();i++){
			g_and_walls->points.push_back(gaussian_sphere_clustered->points[i]);
			g_and_walls->points[0].strength += gaussian_sphere_clustered->points[i].strength;
		}
		sensor_msgs::PointCloud2 ros_g_and_normals;
		pcl::toROSMsg(*g_and_walls, ros_g_and_normals);
		pub.publish(ros_g_and_normals);
	}
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pc_fitting_walls");
	std::cout << "PC FITTING WALLS" << std::endl;
	
	PCFittingWalls pc_fitting_walls;
	ros::spin();
}
