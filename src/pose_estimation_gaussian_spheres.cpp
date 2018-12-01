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

class PoseEstimationGaussianSphere{
	private:
		/*node handle*/
		ros::NodeHandle nh;
		/*subscribe*/
		ros::Subscriber sub_pc;
		ros::Subscriber sub_odom;
		/*publish*/
		ros::Publisher pub_pose;
		ros::Publisher pub_rpy;
		/*struct*/
		struct WallInfo{
			pcl::PointXYZ point;
			nav_msgs::Odometry odom;
			Eigen::MatrixXd X{3, 1};
			Eigen::MatrixXd P{3, 3};
			bool fixed;
			bool found_match;
			int count_match;
			int count_nomatch;
		};
		/*pcl*/
		pcl::visualization::PCLVisualizer viewer{"Gaussian Spheres"};
		pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
		pcl::PointNormal g_vector_from_ekf;
		pcl::PointNormal g_vector_walls;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud {new pcl::PointCloud<pcl::PointXYZ>};
		pcl::PointCloud<pcl::PointNormal>::Ptr normals {new pcl::PointCloud<pcl::PointNormal>};
		pcl::PointCloud<pcl::PointNormal>::Ptr normals_extracted {new pcl::PointCloud<pcl::PointNormal>};
		pcl::PointCloud<pcl::PointXYZ>::Ptr gaussian_sphere {new pcl::PointCloud<pcl::PointXYZ>};
		pcl::PointCloud<pcl::PointXYZ>::Ptr gaussian_sphere_clustered {new pcl::PointCloud<pcl::PointXYZ>};
		pcl::PointCloud<pcl::PointNormal>::Ptr gaussian_sphere_clustered_n {new pcl::PointCloud<pcl::PointNormal>};
		pcl::PointCloud<pcl::PointXYZ>::Ptr gaussian_sphere_clustered_weighted {new pcl::PointCloud<pcl::PointXYZ>};
		pcl::PointCloud<pcl::PointXYZ>::Ptr d_gaussian_sphere {new pcl::PointCloud<pcl::PointXYZ>};
		pcl::PointCloud<pcl::PointXYZ>::Ptr d_gaussian_sphere_clustered {new pcl::PointCloud<pcl::PointXYZ>};
		pcl::PointCloud<pcl::PointNormal>::Ptr d_gaussian_sphere_clustered_n {new pcl::PointCloud<pcl::PointNormal>};
		pcl::PointCloud<pcl::PointXYZ>::Ptr d_gaussian_sphere_registered {new pcl::PointCloud<pcl::PointXYZ>};
		pcl::PointCloud<pcl::PointNormal>::Ptr d_gaussian_sphere_registered_n {new pcl::PointCloud<pcl::PointNormal>};
		/*objects*/
		std::vector<WallInfo> list_walls;
		nav_msgs::Odometry odom_now;
		geometry_msgs::PoseStamped pose_pub;
		std_msgs::Float64MultiArray rpy_pub;
		ros::Time time_pub;
		/*flags*/
		bool first_callback_odom = true;
	public:
		PoseEstimationGaussianSphere();
		void CallbackPC(const sensor_msgs::PointCloud2ConstPtr &msg);
		void CallbackOdom(const nav_msgs::OdometryConstPtr& msg);
		void ConvertionPoseToGVector(void);
		void ClearPoints(void);
		void FittingWalls(void);
		std::vector<int> KdtreeSearch(pcl::PointXYZ searchpoint, double search_radius);
		double AngleBetweenVectors(pcl::PointNormal v1, pcl::PointNormal v2);
		double ComputeSquareError(Eigen::Vector4f plane_parameters, std::vector<int> indices);
		void ClusterGauss(void);
		bool GVectorEstimation(void);
		void PartialRotation(void);
		void ClusterDGauss(void);
		void CreateRegisteredCentroidCloud(void);
		pcl::PointXYZ PointTransformation(pcl::PointXYZ p, nav_msgs::Odometry origin, nav_msgs::Odometry target);
		bool MatchWalls(void);
		void InputNewWallInfo(pcl::PointXYZ p);
		void KalmanFilterForRegistration(WallInfo& wall);
		tf::Quaternion GetRelativeRotation(pcl::PointXYZ orgin, pcl::PointXYZ target);
		void Visualization(void);
		void Publication(void);
};

PoseEstimationGaussianSphere::PoseEstimationGaussianSphere()
{
	sub_pc = nh.subscribe("/velodyne_points", 1, &PoseEstimationGaussianSphere::CallbackPC, this);
	sub_odom = nh.subscribe("/combined_odometry", 1, &PoseEstimationGaussianSphere::CallbackOdom, this);
	pub_rpy = nh.advertise<std_msgs::Float64MultiArray>("/rpy_walls", 1);
	pub_pose = nh.advertise<geometry_msgs::PoseStamped>("/pose_dgauss", 1);
	viewer.setBackgroundColor(1, 1, 1);
	viewer.addCoordinateSystem(0.5, "axis");
	g_vector_from_ekf.x = 0.0;
	g_vector_from_ekf.y = 0.0;
	g_vector_from_ekf.z = 0.0;
	g_vector_walls = g_vector_from_ekf;
	rpy_pub.data.resize(3);
}

void PoseEstimationGaussianSphere::CallbackPC(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	// std::cout << "CALLBACK PC" << std::endl;
	pcl::fromROSMsg(*msg, *cloud);
	time_pub = msg->header.stamp;
	ClearPoints();
	FittingWalls();
	ClusterGauss();
	ClusterDGauss();
	if(!first_callback_odom){
		bool succeeded_rp = GVectorEstimation();
		CreateRegisteredCentroidCloud();
		bool succeeded_y = MatchWalls();
		if(!succeeded_rp){
			rpy_pub.data[0] = NAN;
			rpy_pub.data[1] = NAN;
		}
		if(!succeeded_y)	rpy_pub.data[2] = NAN;
		if(succeeded_rp || succeeded_y)	Publication();
	}
	Visualization();
}

void PoseEstimationGaussianSphere::CallbackOdom(const nav_msgs::OdometryConstPtr& msg)
{
	// std::cout << "CALLBACK ODOM" << std::endl;
	odom_now = *msg;
	ConvertionPoseToGVector();

	first_callback_odom = false;
}

void PoseEstimationGaussianSphere::ConvertionPoseToGVector(void)
{
	tf::Quaternion q_pose_from_ekf;
	quaternionMsgToTF(odom_now.pose.pose.orientation, q_pose_from_ekf);
	tf::Quaternion q_g_vector_global(0.0, 0.0, -1.0, 0.0);
	tf::Quaternion q_g_vector_local = q_pose_from_ekf.inverse()*q_g_vector_global*q_pose_from_ekf;
	g_vector_from_ekf.normal_x = q_g_vector_local.x();
	g_vector_from_ekf.normal_y = q_g_vector_local.y();
	g_vector_from_ekf.normal_z = q_g_vector_local.z();
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
	d_gaussian_sphere->points.clear();
	d_gaussian_sphere_clustered->points.clear();
	d_gaussian_sphere_clustered_n->points.clear();
	d_gaussian_sphere_registered->points.clear();
	d_gaussian_sphere_registered_n->points.clear();
}

void PoseEstimationGaussianSphere::FittingWalls(void)
{
	// std::cout << "NORMAL ESTIMATION" << std::endl;
	kdtree.setInputCloud(cloud);

	const size_t skip_step = 3;
	for(size_t i=0;i<cloud->points.size();i+=skip_step){
		/*search neighbor points*/
		std::vector<int> indices;
		double laser_distance = sqrt(cloud->points[i].x*cloud->points[i].x + cloud->points[i].y*cloud->points[i].y + cloud->points[i].z*cloud->points[i].z);
		const double search_radius_min = 0.3;
		const double ratio = 0.06;
		double search_radius = ratio*laser_distance;
		if(search_radius<search_radius_min)	search_radius = search_radius_min;
		indices = KdtreeSearch(cloud->points[i], search_radius);
		/*judge*/
		const size_t threshold_num_neighborpoints = 5;
		if(indices.size()<threshold_num_neighborpoints){
			// std::cout << ">> indices.size() = " << indices.size() << " < " << threshold_num_neighborpoints << ", then skip" << std::endl;
			continue;
		}
		/*compute normal*/
		float curvature;
		Eigen::Vector4f plane_parameters;
		pcl::computePointNormal(*cloud, indices, plane_parameters, curvature);
		/*create tmp object*/
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
		const double threshold_angle = 30.0;	//[deg]
		if(fabs(AngleBetweenVectors(tmp_normal, g_vector_from_ekf)-M_PI/2.0)>threshold_angle/180.0*M_PI){
			// std::cout << ">> angle from square angle = " << fabs(AngleBetweenVectors(tmp_normal, g_vector_from_ekf->points[0])-M_PI/2.0) << " > " << threshold_angle << ", then skip" << std::endl;
			continue;
		}
		/*judge*/
		const double threshold_fitting_error = 0.01;	//[m]
		if(ComputeSquareError(plane_parameters, indices)>threshold_fitting_error){
			// std::cout << ">> square error = " << ComputeSquareError(plane_parameters, indices) << " > " << threshold_fitting_error << ", then skip" << std::endl;
			continue;
		}
		/*delete nan*/
		if(std::isnan(plane_parameters[0]) || std::isnan(plane_parameters[1]) || std::isnan(plane_parameters[2])){
			// std::cout << ">> this point has NAN, then skip" << std::endl;
			continue;
		}
		/*input*/
		// std::cout << ">> ok, then input" << std::endl;
		normals_extracted->points.push_back(tmp_normal);
		/*unit gaussian sphere*/
		pcl::PointXYZ tmp_point;
		tmp_point.x = plane_parameters[0];
		tmp_point.y = plane_parameters[1];
		tmp_point.z = plane_parameters[2];
		gaussian_sphere->points.push_back(tmp_point);
		/*d-gaussian sphere*/
		tmp_point.x = -plane_parameters[3]*plane_parameters[0];
		tmp_point.y = -plane_parameters[3]*plane_parameters[1];
		tmp_point.z = -plane_parameters[3]*plane_parameters[2];
		d_gaussian_sphere->points.push_back(tmp_point);
	}
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
	double angle = acosf(dot_product/(v1_norm*v2_norm));
	return angle;
}

double PoseEstimationGaussianSphere::ComputeSquareError(Eigen::Vector4f plane_parameters, std::vector<int> indices)
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

void PoseEstimationGaussianSphere::ClusterGauss(void)
{
	// std::cout << "POINT CLUSTER" << std::endl;
	// const double cluster_distance = 0.1;
	const double cluster_distance = 0.1;
	const int min_num_cluster_belongings = 60;
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
		g_vector_walls.normal_x = gaussian_sphere_clustered->points[0].y*gaussian_sphere_clustered->points[1].z - gaussian_sphere_clustered->points[0].z*gaussian_sphere_clustered->points[1].y;
		g_vector_walls.normal_y = gaussian_sphere_clustered->points[0].z*gaussian_sphere_clustered->points[1].x - gaussian_sphere_clustered->points[0].x*gaussian_sphere_clustered->points[1].z;
		g_vector_walls.normal_z = gaussian_sphere_clustered->points[0].x*gaussian_sphere_clustered->points[1].y - gaussian_sphere_clustered->points[0].y*gaussian_sphere_clustered->points[1].x;
	}
	else if(gaussian_sphere_clustered->points.size()>2){
		// std::cout << ">> more than 3 normals" << std::endl;
		Eigen::Vector4f plane_parameters;
		float curvature;
		pcl::computePointNormal(*gaussian_sphere_clustered_weighted, plane_parameters, curvature);
		g_vector_walls.normal_x = plane_parameters[0];
		g_vector_walls.normal_y = plane_parameters[1];
		g_vector_walls.normal_z = plane_parameters[2];
	}
	/*flip*/
	flipNormalTowardsViewpoint(g_vector_walls, 0.0, 0.0, -100.0, g_vector_walls.normal_x, g_vector_walls.normal_y, g_vector_walls.normal_z);
	/*if angle variation is too large, estimation would be wrong*/
	const double threshold_angle_variation = 60.0;	//[deg]
	if(fabs(AngleBetweenVectors(g_vector_walls, g_vector_from_ekf))>threshold_angle_variation/180.0*M_PI){
		std::cout << ">> angle variation of g in 1 step is too large and would be wrong " << std::endl;
		return false;
	}
	/*normalization*/
	double norm_g = sqrt(g_vector_walls.normal_x*g_vector_walls.normal_x + g_vector_walls.normal_y*g_vector_walls.normal_y + g_vector_walls.normal_z*g_vector_walls.normal_z);
	g_vector_walls.normal_x /= norm_g;
	g_vector_walls.normal_y /= norm_g;
	g_vector_walls.normal_z /= norm_g;
	/*convertion to roll, pitch*/
	const double g = -9.80665;
	rpy_pub.data[0] = atan2(g*g_vector_walls.normal_y, g*g_vector_walls.normal_z);
	rpy_pub.data[1] = atan2(-g*g_vector_walls.normal_x, sqrt(g*g_vector_walls.normal_y*g*g_vector_walls.normal_y + g*g_vector_walls.normal_z*g*g_vector_walls.normal_z));
	return true;
}

void PoseEstimationGaussianSphere::PartialRotation(void)
{
	/*projection*/
	double dot_product =
		g_vector_from_ekf.normal_x*gaussian_sphere_clustered->points[0].x
		+ g_vector_from_ekf.normal_y*gaussian_sphere_clustered->points[0].y
		+ g_vector_from_ekf.normal_z*gaussian_sphere_clustered->points[0].z;
	g_vector_walls.normal_x = g_vector_from_ekf.normal_x - dot_product*gaussian_sphere_clustered->points[0].x;
	g_vector_walls.normal_y = g_vector_from_ekf.normal_y - dot_product*gaussian_sphere_clustered->points[0].y;
	g_vector_walls.normal_z = g_vector_from_ekf.normal_z - dot_product*gaussian_sphere_clustered->points[0].z;
}

void PoseEstimationGaussianSphere::ClusterDGauss(void)
{
	// std::cout << "POINT CLUSTER" << std::endl;
	const double cluster_distance = 0.2;
	const int min_num_cluster_belongings = 20;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(d_gaussian_sphere);
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ece;
	ece.setClusterTolerance(cluster_distance);
	ece.setMinClusterSize(min_num_cluster_belongings);
	ece.setMaxClusterSize(d_gaussian_sphere->points.size());
	ece.setSearchMethod(tree);
	ece.setInputCloud(d_gaussian_sphere);
	ece.extract(cluster_indices);

	// std::cout << "cluster_indices.size() = " << cluster_indices.size() << std::endl;

	for(size_t i=0;i<cluster_indices.size();i++){
		/*extract*/
		pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_clustered_points (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointIndices::Ptr tmp_clustered_indices (new pcl::PointIndices);
		*tmp_clustered_indices = cluster_indices[i];
		pcl::ExtractIndices<pcl::PointXYZ> ei;
		ei.setInputCloud(d_gaussian_sphere);
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
		d_gaussian_sphere_clustered->points.push_back(tmp_centroid);
		pcl::PointNormal tmp_centroid_n;
		tmp_centroid_n.x = 0.0;
		tmp_centroid_n.y = 0.0;
		tmp_centroid_n.z = 0.0;
		tmp_centroid_n.normal_x = xyz_centroid[0];
		tmp_centroid_n.normal_y = xyz_centroid[1];
		tmp_centroid_n.normal_z = xyz_centroid[2];
		d_gaussian_sphere_clustered_n->points.push_back(tmp_centroid_n);
	}
}

void PoseEstimationGaussianSphere::CreateRegisteredCentroidCloud(void)
{
	for(size_t i=0;i<list_walls.size();i++){
		d_gaussian_sphere_registered->points.push_back(PointTransformation(list_walls[i].point, list_walls[i].odom, odom_now));

		pcl::PointNormal tmp_normal;
		tmp_normal.x = 0.0;
		tmp_normal.y = 0.0;
		tmp_normal.z = 0.0;
		tmp_normal.normal_x = d_gaussian_sphere_registered->points[i].x;
		tmp_normal.normal_y = d_gaussian_sphere_registered->points[i].y;
		tmp_normal.normal_z = d_gaussian_sphere_registered->points[i].z;
		d_gaussian_sphere_registered_n->points.push_back(tmp_normal);
	}
}

pcl::PointXYZ PoseEstimationGaussianSphere::PointTransformation(pcl::PointXYZ p, nav_msgs::Odometry origin, nav_msgs::Odometry target)
{
	tf::Quaternion q_pose_origin;
	tf::Quaternion q_pose_target;
	quaternionMsgToTF(origin.pose.pose.orientation, q_pose_origin);
	quaternionMsgToTF(target.pose.pose.orientation, q_pose_target);
	/*linear*/
	tf::Quaternion q_global_move(
			target.pose.pose.position.x - origin.pose.pose.position.x,
			target.pose.pose.position.y - origin.pose.pose.position.y,
			target.pose.pose.position.z - origin.pose.pose.position.z,
			0.0);
	tf::Quaternion q_local_move = q_pose_origin.inverse()*q_global_move*q_pose_origin;
	Eigen::Vector3d vec_local_move(q_local_move.x(), q_local_move.y(), q_local_move.z());
	Eigen::Vector3d vec_normal(p.x, p.y, p.z);
	Eigen::Vector3d vec_vertical_local_move = (vec_local_move.dot(vec_normal)/vec_normal.dot(vec_normal))*vec_normal;
	tf::Quaternion q_point_origin(
			p.x - vec_vertical_local_move(0),
			p.y - vec_vertical_local_move(1),
			p.z - vec_vertical_local_move(2),
			0.0);
	/*rotation*/
	tf::Quaternion relative_rotation = q_pose_origin*q_pose_target.inverse();	//inverse rotation to pose change
	relative_rotation.normalize();
	tf::Quaternion q_point_target = relative_rotation*q_point_origin*relative_rotation.inverse();
	/*input*/
	pcl::PointXYZ p_;
	p_.x = q_point_target.x();
	p_.y = q_point_target.y();
	p_.z = q_point_target.z();

	return p_;
}

bool PoseEstimationGaussianSphere::MatchWalls(void)
{
	// std::cout << "MATCH WALLS" << std::endl;

	bool succeeded_y = false;
	double local_pose_error_rpy_sincosatan[3][3] = {};
	tf::Quaternion q_ave_local_pose_error;
	bool compute_local_pose_error_in_quaternion = false;

	std::cout << "list_walls.size() = " << list_walls.size() << " -------------------"<< std::endl;
	if(list_walls.empty()){
		for(size_t i=0;i<d_gaussian_sphere_clustered->points.size();i++) InputNewWallInfo(d_gaussian_sphere_clustered->points[i]);
		return succeeded_y;
	}
	else{
		const double ratio_matching_distance = 0.4;
		const int threshold_count_match = 5;
		const int k = 1;
		kdtree.setInputCloud(d_gaussian_sphere_registered);
		for(size_t i=0;i<d_gaussian_sphere_clustered->points.size();i++){
			std::vector<int> pointIdxNKNSearch(k);
			std::vector<float> pointNKNSquaredDistance(k);
			if(kdtree.nearestKSearch(d_gaussian_sphere_clustered->points[i], k, pointIdxNKNSearch, pointNKNSquaredDistance)<=0)	std::cout << "kdtree error" << std::endl;
			double point_depth = sqrt(d_gaussian_sphere_clustered->points[i].x*d_gaussian_sphere_clustered->points[i].x + d_gaussian_sphere_clustered->points[i].y*d_gaussian_sphere_clustered->points[i].y + d_gaussian_sphere_clustered->points[i].z*d_gaussian_sphere_clustered->points[i].z);
			double threshold_matching_distance = ratio_matching_distance*point_depth;
			if(sqrt(pointNKNSquaredDistance[0])<threshold_matching_distance && !list_walls[pointIdxNKNSearch[0]].found_match){
				list_walls[pointIdxNKNSearch[0]].found_match = true;
				list_walls[pointIdxNKNSearch[0]].count_match++;
				list_walls[pointIdxNKNSearch[0]].count_nomatch = 0;
				if(list_walls[pointIdxNKNSearch[0]].fixed){
					tf::Quaternion tmp_q_local_pose_error = GetRelativeRotation(d_gaussian_sphere_clustered->points[i], d_gaussian_sphere_registered->points[pointIdxNKNSearch[0]]);
					if(compute_local_pose_error_in_quaternion){
						tmp_q_local_pose_error = tf::Quaternion(list_walls[pointIdxNKNSearch[0]].count_match*tmp_q_local_pose_error.x(), list_walls[pointIdxNKNSearch[0]].count_match*tmp_q_local_pose_error.y(), list_walls[pointIdxNKNSearch[0]].count_match*tmp_q_local_pose_error.z(), list_walls[pointIdxNKNSearch[0]].count_match*tmp_q_local_pose_error.w());
						if(!succeeded_y)	q_ave_local_pose_error = tmp_q_local_pose_error;
						else	q_ave_local_pose_error += tmp_q_local_pose_error;
					}
					else{
						double tmp_local_pose_error_rpy[3];
						tf::Matrix3x3(tmp_q_local_pose_error).getRPY(tmp_local_pose_error_rpy[0], tmp_local_pose_error_rpy[1], tmp_local_pose_error_rpy[2]);
						for(int j=0;j<3;j++){
							// local_pose_error_rpy_sincosatan[j][0] += sin(tmp_local_pose_error_rpy[j]);
							// local_pose_error_rpy_sincosatan[j][1] += cos(tmp_local_pose_error_rpy[j]);
							local_pose_error_rpy_sincosatan[j][0] += list_walls[pointIdxNKNSearch[0]].count_match*sin(tmp_local_pose_error_rpy[j]);
							local_pose_error_rpy_sincosatan[j][1] += list_walls[pointIdxNKNSearch[0]].count_match*cos(tmp_local_pose_error_rpy[j]);
							// double distance = sqrt(d_gaussian_sphere_clustered->points[i].x*d_gaussian_sphere_clustered->points[i].x + d_gaussian_sphere_clustered->points[i].y*d_gaussian_sphere_clustered->points[i].y + d_gaussian_sphere_clustered->points[i].z*d_gaussian_sphere_clustered->points[i].z);
							// local_pose_error_rpy_sincosatan[j][0] += distance*sin(tmp_local_pose_error_rpy[j]);
							// local_pose_error_rpy_sincosatan[j][1] += distance*cos(tmp_local_pose_error_rpy[j]);
						}
					}
					succeeded_y = true;
					std::cout << "list_walls[" << pointIdxNKNSearch[0] << "].count_match = " << list_walls[pointIdxNKNSearch[0]].count_match << std::endl;
				}
				else{
					list_walls[pointIdxNKNSearch[0]].point = d_gaussian_sphere_clustered->points[i];
					KalmanFilterForRegistration(list_walls[pointIdxNKNSearch[0]]);
					if(list_walls[pointIdxNKNSearch[0]].count_match>threshold_count_match)	list_walls[pointIdxNKNSearch[0]].fixed = true;
				}
			}
			else	InputNewWallInfo(d_gaussian_sphere_clustered->points[i]);
		}
		/*arrange list*/
		const int threshold_count_nomatch = 10;
		for(size_t i=0;i<list_walls.size();i++){
			if(!list_walls[i].found_match)	list_walls[i].count_nomatch++;
			if(list_walls[i].count_nomatch>threshold_count_nomatch){
				list_walls.erase(list_walls.begin() + i);
				i--;
			}
			else	list_walls[i].found_match = false;
		}
		/*estimate pose*/
		if(succeeded_y){
			if(!compute_local_pose_error_in_quaternion){
				for(int j=0;j<3;j++)	local_pose_error_rpy_sincosatan[j][2] = atan2(local_pose_error_rpy_sincosatan[j][0], local_pose_error_rpy_sincosatan[j][1]);
				q_ave_local_pose_error = tf::createQuaternionFromRPY(local_pose_error_rpy_sincosatan[0][2], local_pose_error_rpy_sincosatan[1][2], local_pose_error_rpy_sincosatan[2][2]);
			}
			q_ave_local_pose_error.normalize();
			tf::Quaternion q_pose_odom_now;
			quaternionMsgToTF(odom_now.pose.pose.orientation, q_pose_odom_now);
			quaternionTFToMsg(q_pose_odom_now*q_ave_local_pose_error, pose_pub.pose.orientation);
			std::cout << "succeeded matching" << std::endl;

			double tmp_rpy[3];
			tf::Matrix3x3(q_pose_odom_now*q_ave_local_pose_error).getRPY(tmp_rpy[0], tmp_rpy[1], tmp_rpy[2]);
			rpy_pub.data[2] = tmp_rpy[2];
			std::cout << "q_ave_local_pose_error: " << tmp_rpy[0] << ", " << tmp_rpy[1] << ", " << tmp_rpy[2] << std::endl;
		}
		return succeeded_y;
	}
}

void PoseEstimationGaussianSphere::InputNewWallInfo(pcl::PointXYZ p)
{
	// std::cout << "INPUT NEW WALL INFO" << std::endl;
	WallInfo tmp_wallinfo;
	tmp_wallinfo.point = p;
	tmp_wallinfo.odom = odom_now;
	tmp_wallinfo.X <<	p.x,
						p.y,
						p.z;
	tmp_wallinfo.P = 1.0e-1*Eigen::MatrixXd::Identity(3, 3);
	tmp_wallinfo.fixed = false;
	tmp_wallinfo.found_match = false;
	tmp_wallinfo.count_match = 0;
	tmp_wallinfo.count_nomatch = 0;
	list_walls.push_back(tmp_wallinfo);
}

void PoseEstimationGaussianSphere::KalmanFilterForRegistration(WallInfo& wall)
{
	// std::cout << "KALMAN FILTER FOR REGISTRATION" << std::endl;
	const int num_state = 3;
	/*prediction*/
	Eigen::MatrixXd A = Eigen::MatrixXd::Identity(num_state, num_state);
	Eigen::MatrixXd F(num_state, 1);
	Eigen::MatrixXd jF = Eigen::MatrixXd::Identity(num_state, num_state);
	const double sigma_pre = 1.0e-1;
	Eigen::MatrixXd Q = sigma_pre*Eigen::MatrixXd::Identity(num_state, num_state);
	F = A*wall.X;
	wall.X = F;
	wall.P = jF*wall.P*jF.transpose() + Q;
	/*observation*/
	const int num_obs = 3;
	Eigen::MatrixXd Z(num_obs, 1);
	pcl::PointXYZ p = PointTransformation(wall.point, odom_now, wall.odom);
	Z <<	p.x,
	  		p.y,
			p.z;
	Eigen::MatrixXd H = Eigen::MatrixXd::Identity(num_obs, num_state);
	Eigen::MatrixXd jH = Eigen::MatrixXd::Identity(num_obs, num_state);
	const double sigma_obs = 2.0e-1;
	Eigen::MatrixXd R = sigma_obs*Eigen::MatrixXd::Identity(num_obs, num_obs);
	Eigen::MatrixXd Y(num_obs, 1);
	Eigen::MatrixXd S(num_obs, num_obs);
	Eigen::MatrixXd K(num_state, num_obs);
	Eigen::MatrixXd I = Eigen::MatrixXd::Identity(num_state, num_state);
	Y = Z - H*wall.X;
	S = jH*wall.P*jH.transpose() + R;
	K = wall.P*jH.transpose()*S.inverse();
	wall.X = wall.X + K*Y;
	wall.P = (I - K*jH)*wall.P;

	wall.point.x = wall.X(0, 0);
	wall.point.y = wall.X(1, 0);
	wall.point.z = wall.X(2, 0);

	std::cout << "Y  : (" << Y(0, 0) << ", " << Y(1, 0) << ", " << Y(2, 0) << ")" << std::endl;
	std::cout << "K*Y: (" << (K*Y)(0, 0) << ", " << (K*Y)(1, 0) << ", " << (K*Y)(2, 0) << ")" << std::endl;
}

tf::Quaternion PoseEstimationGaussianSphere::GetRelativeRotation(pcl::PointXYZ origin, pcl::PointXYZ target)
{
	Eigen::Vector3d Origin(origin.x, origin.y, origin.z);
	Eigen::Vector3d Target(target.x, target.y, target.z);
	double theta = acos(Origin.dot(Target)/Origin.norm()/Target.norm());
	Eigen::Vector3d Axis = Origin.cross(Target);
	Axis.normalize();
	tf::Quaternion relative_rotation(sin(theta/2.0)*Axis(0), sin(theta/2.0)*Axis(1), sin(theta/2.0)*Axis(2), cos(theta/2.0));
	relative_rotation.normalize();
	
	double rpy[3];
	tf::Matrix3x3(relative_rotation).getRPY(rpy[0], rpy[1], rpy[2]);
	std::cout << "local error: " << rpy[0] << ", " << rpy[1] << ", " << rpy[2] << std::endl;
	std::cout << "distance: " << (Target - Origin).norm() << std::endl;
	std::cout << "Origin: (" << Origin(0) << ", " << Origin(1) << ", " << Origin(2) << "), depth = " << Origin.norm() << std::endl;
	std::cout << "Target: (" << Target(0) << ", " << Target(1) << ", " << Target(2) << "), depth = " << Target.norm() << std::endl;

	return relative_rotation;
}

void PoseEstimationGaussianSphere::Visualization(void)
{
	viewer.removeAllPointClouds();

	viewer.addPointCloud(cloud, "cloud");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 0.0, "cloud");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
	
	viewer.addPointCloudNormals<pcl::PointNormal>(normals, 1, 0.5, "normals");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "normals");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, "normals");
	
	viewer.addPointCloudNormals<pcl::PointNormal>(normals_extracted, 1, 0.5, "normals_extracted");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 1.0, "normals_extracted");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, "normals_extracted");

	viewer.addPointCloud(gaussian_sphere, "gaussian_sphere");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "gaussian_sphere");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "gaussian_sphere");

	viewer.addPointCloudNormals<pcl::PointNormal>(gaussian_sphere_clustered_n, 1, 1.0, "gaussian_sphere_clustered_n");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.5, 0.0, "gaussian_sphere_clustered_n");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "gaussian_sphere_clustered_n");

	viewer.addPointCloud(d_gaussian_sphere, "d_gaussian_sphere");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.8, "d_gaussian_sphere");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "d_gaussian_sphere");
	
	viewer.addPointCloudNormals<pcl::PointNormal>(d_gaussian_sphere_clustered_n, 1, 1.0, "d_gaussian_sphere_clustered_n");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.8, 0.0, 1.0, "d_gaussian_sphere_clustered_n");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1, "d_gaussian_sphere_clustered_n");
	
	viewer.addPointCloudNormals<pcl::PointNormal>(d_gaussian_sphere_registered_n, 1, 1.0, "d_gaussian_sphere_registered_n");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.5, 0.0, "d_gaussian_sphere_registered_n");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1, "d_gaussian_sphere_registered_n");

	pcl::PointCloud<pcl::PointNormal>::Ptr tmp_g_vector_walls {new pcl::PointCloud<pcl::PointNormal>};
	tmp_g_vector_walls->points.push_back(g_vector_walls);
	viewer.addPointCloudNormals<pcl::PointNormal>(tmp_g_vector_walls, 1, 1.0, "g_vector_walls");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "g_vector_walls");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, "g_vector_walls");
	
	viewer.spinOnce();
}

void PoseEstimationGaussianSphere::Publication(void)
{
	pub_rpy.publish(rpy_pub);

	pose_pub.header.frame_id = odom_now.header.frame_id;
	pose_pub.header.stamp = time_pub;
	pub_pose.publish(pose_pub);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pose_estimation_gaussian_sphere");
	
	PoseEstimationGaussianSphere pose_estimation_gaussian_sphere;

	ros::spin();
}