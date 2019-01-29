#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <tf/tf.h>
#include <thread>
#include <omp.h>

class PoseEstimationGaussianSphere{
	private:
		/*node handle*/
		ros::NodeHandle nh;
		/*subscribe*/
		ros::Subscriber sub_pc;
		/*publish*/
		ros::Publisher pub_pose;
		ros::Publisher pub_rpy;
		/*pcl*/
		pcl::visualization::PCLVisualizer viewer{"Gaussian Spheres"};
		pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
		pcl::PointNormal g_vector_last;
		pcl::PointNormal g_vector_new;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud {new pcl::PointCloud<pcl::PointXYZ>};
		pcl::PointCloud<pcl::PointNormal>::Ptr normals {new pcl::PointCloud<pcl::PointNormal>};
		pcl::PointCloud<pcl::PointNormal>::Ptr normals_extracted {new pcl::PointCloud<pcl::PointNormal>};
		pcl::PointCloud<pcl::PointXYZ>::Ptr gaussian_sphere {new pcl::PointCloud<pcl::PointXYZ>};
		pcl::PointCloud<pcl::PointXYZ>::Ptr gaussian_sphere_clustered {new pcl::PointCloud<pcl::PointXYZ>};
		pcl::PointCloud<pcl::PointNormal>::Ptr gaussian_sphere_clustered_n {new pcl::PointCloud<pcl::PointNormal>};
		pcl::PointCloud<pcl::PointXYZ>::Ptr gaussian_sphere_clustered_weighted {new pcl::PointCloud<pcl::PointXYZ>};
		/*objects*/
		tf::Quaternion q_pose = {0.0, 0.0, 0.0, 1.0};
		geometry_msgs::PoseStamped pose_pub;
		std_msgs::Float64MultiArray rpy_pub;
		ros::Time time_pub;
	public:
		PoseEstimationGaussianSphere();
		void CallbackPC(const sensor_msgs::PointCloud2ConstPtr &msg);
		void ClearPoints(void);
		void FittingWalls(void);
		std::vector<int> KdtreeSearch(pcl::PointXYZ searchpoint, double search_radius);
		double AngleBetweenVectors(pcl::PointNormal v1, pcl::PointNormal v2);
		double ComputeSquareError(Eigen::Vector4f plane_parameters, std::vector<int> indices);
		void ClusterGauss(void);
		bool GVectorEstimation(void);
		void PartialRotation(void);
		tf::Quaternion GetRelativeRotationNormals(pcl::PointNormal origin, pcl::PointNormal target);
		void FinalEstimation(void);
		void Visualization(void);
		void Publication(void);
	protected:
		class FittingWalls_{
			private:
				pcl::PointCloud<pcl::PointNormal>::Ptr normals_ {new pcl::PointCloud<pcl::PointNormal>};
				pcl::PointCloud<pcl::PointNormal>::Ptr normals_extracted_ {new pcl::PointCloud<pcl::PointNormal>};
				pcl::PointCloud<pcl::PointXYZ>::Ptr gaussian_sphere_ {new pcl::PointCloud<pcl::PointXYZ>};
			public:
				void Compute(PoseEstimationGaussianSphere &mainclass, size_t i_start, size_t i_end);
				void Merge(PoseEstimationGaussianSphere &mainclass);
		};
};

PoseEstimationGaussianSphere::PoseEstimationGaussianSphere()
{
	sub_pc = nh.subscribe("/velodyne_points", 1, &PoseEstimationGaussianSphere::CallbackPC, this);
	pub_rpy = nh.advertise<std_msgs::Float64MultiArray>("/rpy_walls", 1);
	pub_pose = nh.advertise<geometry_msgs::PoseStamped>("/pose_dgauss", 1);
	viewer.setBackgroundColor(1, 1, 1);
	viewer.addCoordinateSystem(0.8, "axis");
	// viewer.setCameraPosition(-6.0, -6.0, 6.0, 0.0, 0.0, 1.0);
	// viewer.setCameraPosition(0.0, 0.0, 40.0, 0.0, 0.0, 0.0);
	viewer.setCameraPosition(35.0, -15.0, 25.0, 0.0, 0.0, 1.0);
	g_vector_last.x = 0.0;
	g_vector_last.y = 0.0;
	g_vector_last.z = 0.0;
	g_vector_last.normal_x = 0.0;
	g_vector_last.normal_y = 0.0;
	g_vector_last.normal_z = -1.0;
	g_vector_new = g_vector_last;
	rpy_pub.data.resize(3);
}

void PoseEstimationGaussianSphere::CallbackPC(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	// std::cout << "CALLBACK PC" << std::endl;
	pcl::fromROSMsg(*msg, *cloud);
	time_pub = msg->header.stamp;
	ClearPoints();
	kdtree.setInputCloud(cloud);
	const int num_threads = std::thread::hardware_concurrency();
	std::cout << "std::thread::hardware_concurrency() = " << std::thread::hardware_concurrency() << std::endl;
	std::cout << "cloud->points.size() = " << cloud->points.size() << std::endl;
	std::vector<std::thread> threads_fittingwalls;
	std::vector<FittingWalls_> objects;
	for(int i=0;i<num_threads;i++){
		FittingWalls_ tmp_object;
		objects.push_back(tmp_object);
	}
	for(int i=0;i<num_threads;i++){
		threads_fittingwalls.push_back(
			std::thread([i, num_threads, &objects, this]{
				objects[i].Compute(*this, i*cloud->points.size()/num_threads, (i+1)*cloud->points.size()/num_threads);
			})
		);
	}
	for(std::thread &th : threads_fittingwalls)	th.join();
	for(int i=0;i<num_threads;i++)	objects[i].Merge(*this);

	ClusterGauss();
	bool succeeded_rp = GVectorEstimation();

	if(succeeded_rp){
		FinalEstimation();
		Publication();
	}
	Visualization();
	g_vector_last = g_vector_new;
}

void PoseEstimationGaussianSphere::ClearPoints(void)
{
	// std::cout << "CLEAR POINTS" << std::endl;
	normals->points.clear();
	normals_extracted->points.clear();
	gaussian_sphere->points.clear();
	gaussian_sphere_clustered->points.clear();
	gaussian_sphere_clustered_n->points.clear();
	gaussian_sphere_clustered_weighted->points.clear();
}

void PoseEstimationGaussianSphere::FittingWalls_::Compute(PoseEstimationGaussianSphere &mainclass, size_t i_start, size_t i_end)
{
	// std::cout << "NORMAL ESTIMATION" << std::endl;

	const size_t skip_step = 3;
	for(size_t i=i_start;i<i_end;i+=skip_step){
		bool input_to_gauss = true;
		/*search neighbor points*/
		std::vector<int> indices;
		double laser_distance = sqrt(mainclass.cloud->points[i].x*mainclass.cloud->points[i].x + mainclass.cloud->points[i].y*mainclass.cloud->points[i].y + mainclass.cloud->points[i].z*mainclass.cloud->points[i].z);
		const double search_radius_min = 0.3;
		const double ratio = 0.09;
		double search_radius = ratio*laser_distance;
		if(search_radius<search_radius_min)	search_radius = search_radius_min;
		indices = mainclass.KdtreeSearch(mainclass.cloud->points[i], search_radius);
		/*judge*/
		const size_t threshold_num_neighborpoints_gauss = 20;
		if(indices.size()<threshold_num_neighborpoints_gauss)	continue;
		/*compute normal*/
		float curvature;
		Eigen::Vector4f plane_parameters;
		pcl::computePointNormal(*mainclass.cloud, indices, plane_parameters, curvature);
		/*create tmp object*/
		pcl::PointNormal tmp_normal;
		tmp_normal.x = mainclass.cloud->points[i].x;
		tmp_normal.y = mainclass.cloud->points[i].y;
		tmp_normal.z = mainclass.cloud->points[i].z;
		tmp_normal.normal_x = plane_parameters[0];
		tmp_normal.normal_y = plane_parameters[1];
		tmp_normal.normal_z = plane_parameters[2];
		tmp_normal.curvature = curvature;
		flipNormalTowardsViewpoint(tmp_normal, 0.0, 0.0, 0.0, tmp_normal.normal_x, tmp_normal.normal_y, tmp_normal.normal_z);
		normals_->points.push_back(tmp_normal);
		/*delete nan*/
		if(std::isnan(plane_parameters[0]) || std::isnan(plane_parameters[1]) || std::isnan(plane_parameters[2]))	continue;
		/*judge*/
		const double threshold_angle = 40.0;	//[deg]
		if(fabs(fabs(mainclass.AngleBetweenVectors(tmp_normal, mainclass.g_vector_last))-M_PI/2.0)>threshold_angle/180.0*M_PI)	continue;
		/*judge*/
		const double threshold_fitting_error = 0.01;	//[m]
		if(mainclass.ComputeSquareError(plane_parameters, indices)>threshold_fitting_error)	continue;
		/*input*/
		normals_extracted_->points.push_back(tmp_normal);
		/*unit gaussian sphere*/
		pcl::PointXYZ tmp_point;
		tmp_point.x = plane_parameters[0];
		tmp_point.y = plane_parameters[1];
		tmp_point.z = plane_parameters[2];
		gaussian_sphere_->points.push_back(tmp_point);
	}
}

void PoseEstimationGaussianSphere::FittingWalls_::Merge(PoseEstimationGaussianSphere &mainclass)
{
	*mainclass.normals += *normals_;
	*mainclass.normals_extracted += *normals_extracted_;
	*mainclass.gaussian_sphere += *gaussian_sphere_;
}

std::vector<int> PoseEstimationGaussianSphere::KdtreeSearch(pcl::PointXYZ searchpoint, double search_radius)
{
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;
	if(kdtree.radiusSearch(searchpoint, search_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance)<=0)	std::cout << "kdtree error" << std::endl;
	return pointIdxRadiusSearch; 
}

double PoseEstimationGaussianSphere::AngleBetweenVectors(pcl::PointNormal v1, pcl::PointNormal v2)
{
	double dot_product = v1.normal_x*v2.normal_x + v1.normal_y*v2.normal_y + v1.normal_z*v2.normal_z;
	double v1_norm = sqrt(v1.normal_x*v1.normal_x + v1.normal_y*v1.normal_y + v1.normal_z*v1.normal_z);
	double v2_norm = sqrt(v2.normal_x*v2.normal_x + v2.normal_y*v2.normal_y + v2.normal_z*v2.normal_z);
	double angle = acos(dot_product/(v1_norm*v2_norm));
	return angle;
}

double PoseEstimationGaussianSphere::ComputeSquareError(Eigen::Vector4f plane_parameters, std::vector<int> indices)
{
	double sum_square_error = 0.0;
	for(size_t i=0;i<indices.size();i++){
		sum_square_error +=		fabs(plane_parameters[0]*cloud->points[indices[i]].x
									+plane_parameters[1]*cloud->points[indices[i]].y
									+plane_parameters[2]*cloud->points[indices[i]].z
									+plane_parameters[3])
								/sqrt(plane_parameters[0]*plane_parameters[0]
									+plane_parameters[1]*plane_parameters[1]
									+plane_parameters[2]*plane_parameters[2])
								/(double)indices.size();
	}
	return sum_square_error;
}

void PoseEstimationGaussianSphere::ClusterGauss(void)
{
	// std::cout << "POINT CLUSTER" << std::endl;
	const double cluster_distance = 0.1;
	const int min_num_cluster_belongings = 70;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(gaussian_sphere);
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ece;
	ece.setClusterTolerance(cluster_distance);
	ece.setMinClusterSize(min_num_cluster_belongings);
	ece.setMaxClusterSize(gaussian_sphere->points.size());
	ece.setSearchMethod(tree);
	ece.setInputCloud(gaussian_sphere);
	ece.extract(cluster_indices);

	// std::cout << "cluster_indices.size() = " << cluster_indices.size() << std::endl;

	for(size_t i=0;i<cluster_indices.size();i++){
		/*extract*/
		pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_clustered_points (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointIndices::Ptr tmp_clustered_indices (new pcl::PointIndices);
		*tmp_clustered_indices = cluster_indices[i];
		pcl::ExtractIndices<pcl::PointXYZ> ei;
		ei.setInputCloud(gaussian_sphere);
		ei.setIndices(tmp_clustered_indices);
		ei.setNegative(false);
		ei.filter(*tmp_clustered_points);
		/*compute centroid*/
		Eigen::Vector4f xyz_centroid;
		pcl::compute3DCentroid(*tmp_clustered_points, xyz_centroid);
		/*input*/
		pcl::PointXYZ tmp_centroid;
		tmp_centroid.x = xyz_centroid[0];
		tmp_centroid.y = xyz_centroid[1];
		tmp_centroid.z = xyz_centroid[2];
		gaussian_sphere_clustered->points.push_back(tmp_centroid);
		tmp_centroid.x = cluster_indices[i].indices.size()*xyz_centroid[0];
		tmp_centroid.y = cluster_indices[i].indices.size()*xyz_centroid[1];
		tmp_centroid.z = cluster_indices[i].indices.size()*xyz_centroid[2];
		gaussian_sphere_clustered_weighted->points.push_back(tmp_centroid);
		/*for the Visualization*/
		pcl::PointNormal tmp_centroid_n;
		tmp_centroid_n.x = 0.0;
		tmp_centroid_n.y = 0.0;
		tmp_centroid_n.z = 0.0;
		tmp_centroid_n.normal_x = xyz_centroid[0];
		tmp_centroid_n.normal_y = xyz_centroid[1];
		tmp_centroid_n.normal_z = xyz_centroid[2];
		gaussian_sphere_clustered_n->points.push_back(tmp_centroid_n);
	}
}

bool PoseEstimationGaussianSphere::GVectorEstimation(void)
{
	// std::cout << "G VECTOR ESTIMATION" << std::endl;
	if(gaussian_sphere_clustered->points.size()==0){
		// std::cout << ">> 0 normal" << std::endl;
		return false;
	}
	else if(gaussian_sphere_clustered->points.size()==1){
		// std::cout << ">> 1 normal" << std::endl;
		PartialRotation();
	}
	else if(gaussian_sphere_clustered->points.size()==2){
		// std::cout << ">> 2 normals" << std::endl;
		/*cross product*/
		g_vector_new.normal_x = gaussian_sphere_clustered->points[0].y*gaussian_sphere_clustered->points[1].z - gaussian_sphere_clustered->points[0].z*gaussian_sphere_clustered->points[1].y;
		g_vector_new.normal_y = gaussian_sphere_clustered->points[0].z*gaussian_sphere_clustered->points[1].x - gaussian_sphere_clustered->points[0].x*gaussian_sphere_clustered->points[1].z;
		g_vector_new.normal_z = gaussian_sphere_clustered->points[0].x*gaussian_sphere_clustered->points[1].y - gaussian_sphere_clustered->points[0].y*gaussian_sphere_clustered->points[1].x;
	}
	else if(gaussian_sphere_clustered->points.size()>2){
		// std::cout << ">> more than 3 normals" << std::endl;
		Eigen::Vector4f plane_parameters;
		float curvature;
		pcl::computePointNormal(*gaussian_sphere_clustered_weighted, plane_parameters, curvature);
		g_vector_new.normal_x = plane_parameters[0];
		g_vector_new.normal_y = plane_parameters[1];
		g_vector_new.normal_z = plane_parameters[2];
	}
	/*flip*/
	flipNormalTowardsViewpoint(g_vector_new, 0.0, 0.0, -100.0, g_vector_new.normal_x, g_vector_new.normal_y, g_vector_new.normal_z);
	/*if angle variation is too large, estimation would be wrong*/
	const double threshold_angle_variation = 30.0;	//[deg]
	if(fabs(AngleBetweenVectors(g_vector_new, g_vector_last))>threshold_angle_variation/180.0*M_PI){
		std::cout << ">> angle variation of g in 1 step is too large and would be wrong " << std::endl;
		return false;
	}

	return true;
}

void PoseEstimationGaussianSphere::PartialRotation(void)
{
	/*projection*/
	double dot_product =
		g_vector_last.normal_x*gaussian_sphere_clustered->points[0].x
		+ g_vector_last.normal_y*gaussian_sphere_clustered->points[0].y
		+ g_vector_last.normal_z*gaussian_sphere_clustered->points[0].z;
	g_vector_new.normal_x = g_vector_last.normal_x - dot_product*gaussian_sphere_clustered->points[0].x;
	g_vector_new.normal_y = g_vector_last.normal_y - dot_product*gaussian_sphere_clustered->points[0].y;
	g_vector_new.normal_z = g_vector_last.normal_z - dot_product*gaussian_sphere_clustered->points[0].z;
}

tf::Quaternion PoseEstimationGaussianSphere::GetRelativeRotationNormals(pcl::PointNormal origin, pcl::PointNormal target)
{
	Eigen::Vector3d Origin(origin.normal_x, origin.normal_y, origin.normal_z);
	Eigen::Vector3d Target(target.normal_x, target.normal_y, target.normal_z);
	double theta = acos(Origin.dot(Target)/Origin.norm()/Target.norm());
	Eigen::Vector3d Axis = Origin.cross(Target);
	Axis.normalize();
	tf::Quaternion relative_rotation(sin(theta/2.0)*Axis(0), sin(theta/2.0)*Axis(1), sin(theta/2.0)*Axis(2), cos(theta/2.0));
	relative_rotation.normalize();

	return relative_rotation;
}

void PoseEstimationGaussianSphere::FinalEstimation(void)
{
	tf::Quaternion q_rp_correction = GetRelativeRotationNormals(g_vector_last, g_vector_new).inverse();
	q_pose = q_pose*q_rp_correction;
	tf::Matrix3x3(q_pose).getRPY(rpy_pub.data[0], rpy_pub.data[1], rpy_pub.data[2]);
}

void PoseEstimationGaussianSphere::Visualization(void)
{
	viewer.removeAllPointClouds();

	viewer.addPointCloud(cloud, "cloud");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 0.0, "cloud");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");

	viewer.addPointCloudNormals<pcl::PointNormal>(normals, 1, 0.5, "normals");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "normals");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1, "normals");

	viewer.addPointCloudNormals<pcl::PointNormal>(normals_extracted, 1, 0.5, "normals_extracted");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 1.0, "normals_extracted");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, "normals_extracted");

	viewer.addPointCloud(gaussian_sphere, "gaussian_sphere");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "gaussian_sphere");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "gaussian_sphere");

	viewer.addPointCloudNormals<pcl::PointNormal>(gaussian_sphere_clustered_n, 1, 1.0, "gaussian_sphere_clustered_n");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.5, 0.0, "gaussian_sphere_clustered_n");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "gaussian_sphere_clustered_n");

	pcl::PointCloud<pcl::PointNormal>::Ptr tmp_g_vector_walls (new pcl::PointCloud<pcl::PointNormal>);
	tmp_g_vector_walls->points.push_back(g_vector_new);
	viewer.addPointCloudNormals<pcl::PointNormal>(tmp_g_vector_walls, 1, 1.0, "g_vector_new");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.8, 0.8, 0.0, "g_vector_new");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, "g_vector_new");
	
	viewer.spinOnce();
}

void PoseEstimationGaussianSphere::Publication(void)
{
	pub_rpy.publish(rpy_pub);

	pose_pub.header.frame_id = "/odom";
	pose_pub.header.stamp = time_pub;
	pub_pose.publish(pose_pub);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pose_estimation_gaussian_sphere");
	
	PoseEstimationGaussianSphere pose_estimation_gaussian_sphere;

	ros::spin();
}
