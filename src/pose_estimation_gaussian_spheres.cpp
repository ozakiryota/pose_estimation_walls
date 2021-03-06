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
		ros::Subscriber sub_odom;
		ros::Subscriber sub_inipose;
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
		// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud {new pcl::PointCloud<pcl::PointXYZ>};
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
		pcl::PointCloud<pcl::PointNormal>::Ptr d_gaussian_sphere_newregistered_n {new pcl::PointCloud<pcl::PointNormal>};
		/*objects*/
		std::vector<WallInfo> list_walls;
		std::vector<size_t> list_num_dgauss_cluster_belongings;
		nav_msgs::Odometry odom_now;
		geometry_msgs::PoseStamped pose_pub;
		std_msgs::Float64MultiArray rpy_cov_pub;
		double rp_sincos_calibration[2][2] = {};
		Eigen::Quaternionf lidar_alignment{1.0, 0.0, 0.0, 0.0};
		ros::Time time_pub;
		tf::Quaternion q_rp_correction;
		tf::Quaternion q_y_correction;
		std::vector<int> cases_counter;
		/*flags*/
		bool first_callback_odom = true;
		bool inipose_is_available = false;
		const bool mode_no_d_gauss = true;
	public:
		PoseEstimationGaussianSphere();
		void CallbackPC(const sensor_msgs::PointCloud2ConstPtr &msg);
		void CallbackOdom(const nav_msgs::OdometryConstPtr& msg);
		pcl::PointNormal ConvertionPoseToGVector(tf::Quaternion q_pose);
		void CallbackInipose(const geometry_msgs::QuaternionConstPtr& msg);
		void ClearPoints(void);
		void FittingWalls(void);
		std::vector<int> KdtreeSearch(pcl::PointXYZ searchpoint, double search_radius);
		double AngleBetweenVectors(pcl::PointNormal v1, pcl::PointNormal v2);
		double ComputeSquareError(Eigen::Vector4f plane_parameters, std::vector<int> indices);
		void ClusterGauss(void);
		bool GVectorEstimation(void);
		double ComputeLikelihood(void);
		void PartialRotation(void);
		tf::Quaternion GetRelativeRotationNormals(pcl::PointNormal origin, pcl::PointNormal target);
		void ClusterDGauss(void);
		void CreateRegisteredCentroidCloud(void);
		pcl::PointXYZ PointTransformation(pcl::PointXYZ p, nav_msgs::Odometry origin, nav_msgs::Odometry target);
		bool MatchWalls(void);
		void InputNewWallInfo(pcl::PointXYZ p);
		void KalmanFilterForRegistration(WallInfo& wall);
		tf::Quaternion GetRelativeRotation(pcl::PointXYZ orgin, pcl::PointXYZ target);
		void FinalEstimation(bool succeeded_rp, bool succeeded_y);
		void Visualization(void);
		void Publication(void);
	protected:
		class FittingWalls_{
			private:
				pcl::PointCloud<pcl::PointNormal>::Ptr normals_ {new pcl::PointCloud<pcl::PointNormal>};
				pcl::PointCloud<pcl::PointNormal>::Ptr normals_extracted_ {new pcl::PointCloud<pcl::PointNormal>};
				pcl::PointCloud<pcl::PointXYZ>::Ptr gaussian_sphere_ {new pcl::PointCloud<pcl::PointXYZ>};
				pcl::PointCloud<pcl::PointXYZ>::Ptr d_gaussian_sphere_ {new pcl::PointCloud<pcl::PointXYZ>};
			public:
				void Compute(PoseEstimationGaussianSphere &mainclass, size_t i_start, size_t i_end);
				void Merge(PoseEstimationGaussianSphere &mainclass);
		};
};

PoseEstimationGaussianSphere::PoseEstimationGaussianSphere()
{
	sub_pc = nh.subscribe("/velodyne_points", 1, &PoseEstimationGaussianSphere::CallbackPC, this);
	sub_odom = nh.subscribe("/combined_odometry", 1, &PoseEstimationGaussianSphere::CallbackOdom, this);
	sub_inipose = nh.subscribe("/initial_pose", 1, &PoseEstimationGaussianSphere::CallbackInipose, this);
	pub_rpy = nh.advertise<std_msgs::Float64MultiArray>("/rpy_cov_walls", 1);
	pub_pose = nh.advertise<geometry_msgs::PoseStamped>("/pose_dgauss", 1);
	viewer.setBackgroundColor(1, 1, 1);
	viewer.addCoordinateSystem(0.8, "axis");
	// viewer.setCameraPosition(0.0, 0.0, 50.0, 0.0, 0.0, 0.0);
	viewer.setCameraPosition(-30.0, 0.0, 10.0, 0.0, 0.0, 1.0);
	g_vector_from_ekf.x = 0.0;
	g_vector_from_ekf.y = 0.0;
	g_vector_from_ekf.z = 0.0;
	g_vector_walls = g_vector_from_ekf;
	rpy_cov_pub.data.resize(4);
	cases_counter = {0,0,0,0,0,0};	// 0/1/2/more than 3/exception/all
}

void PoseEstimationGaussianSphere::CallbackPC(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	// std::cout << "CALLBACK PC" << std::endl;
	pcl::fromROSMsg(*msg, *cloud);
	std::cout << "number of points in 1 scan = " << cloud->points.size() << std::endl;
	time_pub = msg->header.stamp;
	if(inipose_is_available){
		pcl::transformPointCloud(*cloud, *cloud, Eigen::Vector3f(0.0, 0.0, 0.0), lidar_alignment);
		cases_counter[5]++;
		std::cout << "cases 0:" << cases_counter[0]/(double)cases_counter[5]*100.0 << "% 1:" << cases_counter[1]/(double)cases_counter[5]*100.0 << "% 2:" << cases_counter[2]/(double)cases_counter[5]*100.0 << "% 3:" << cases_counter[3]/(double)cases_counter[5]*100.0 << "%" << " exp.:" << cases_counter[4]/(double)cases_counter[5]*100.0 << "%" << std::endl;
	}
	ClearPoints();
	kdtree.setInputCloud(cloud);
	const int num_threads = std::thread::hardware_concurrency();
	// std::cout << "std::thread::hardware_concurrency() = " << std::thread::hardware_concurrency() << std::endl;
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
	
	/*test*/
	std::cout << "gaussian_sphere->points.size() = " << gaussian_sphere->points.size() << std::endl;
	const size_t max_num_points = 800;
	if(gaussian_sphere->points.size()>max_num_points){
		/*simple*/
		double sparse_step = gaussian_sphere->points.size()/(double)max_num_points;
		pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud {new pcl::PointCloud<pcl::PointXYZ>};
		for(double a=0.0;a<gaussian_sphere->points.size();a+=sparse_step)	tmp_cloud->points.push_back(gaussian_sphere->points[a]);
		gaussian_sphere = tmp_cloud;

		// #<{(|multi threads|)}>#
		// double sparse_step = gaussian_sphere->points.size()/(double)max_num_points;
		// class DecimatingPoints{
		// 	private:
		// 	public:
		// 		pcl::PointCloud<pcl::PointXYZ>::Ptr pc_decimated {new pcl::PointCloud<pcl::PointXYZ>};
		// 		void PushBack(pcl::PointCloud<pcl::PointXYZ> pc, int start, int end, double step){
		// 			for(double k=start;k<end;k+=step)	pc_decimated->points.push_back(pc.points[k]);
		// 		};
		// };
		// std::vector<DecimatingPoints> decimating_objects;
		// for(int i=0;i<num_threads;i++){
		// 	DecimatingPoints tmp_decimating_object;
		// 	decimating_objects.push_back(tmp_decimating_object);
		// }
		// std::vector<std::thread> decimating_threads;
		// for(int i=0;i<num_threads;i++){
		// 	decimating_threads.push_back(
		// 		std::thread([i, num_threads, &decimating_objects, this, sparse_step]{
		// 			decimating_objects[i].PushBack(*gaussian_sphere, i*gaussian_sphere->points.size()/num_threads, (i+1)*gaussian_sphere->points.size()/num_threads, sparse_step);
		// 		})
		// 	);
		// }
		// for(std::thread &th : decimating_threads)	th.join();
		// gaussian_sphere->points.clear();
		// for(int i=0;i<num_threads;i++)	*gaussian_sphere += *decimating_objects[i].pc_decimated;
		// std::cout << "gaussian_sphere->points.size() = " << gaussian_sphere->points.size() << std::endl;
	}

	if(!first_callback_odom){
		bool succeeded_rp;
		bool succeeded_y = false;
		auto thread_rp = std::thread([&succeeded_rp, this]{
			ClusterGauss();
			succeeded_rp = GVectorEstimation();
		});
		if(!mode_no_d_gauss){
			ClusterDGauss();
			CreateRegisteredCentroidCloud();
			succeeded_y = MatchWalls();
		}
		thread_rp.join();

		if(succeeded_rp || succeeded_y){
			FinalEstimation(succeeded_rp, succeeded_y);
			Publication();
		}
	}
	Visualization();
}

void PoseEstimationGaussianSphere::CallbackOdom(const nav_msgs::OdometryConstPtr& msg)
{
	// std::cout << "CALLBACK ODOM" << std::endl;
	odom_now = *msg;
	tf::Quaternion q_pose_from_ekf;
	quaternionMsgToTF(odom_now.pose.pose.orientation, q_pose_from_ekf);
	g_vector_from_ekf = ConvertionPoseToGVector(q_pose_from_ekf);

	first_callback_odom = false;
}

pcl::PointNormal PoseEstimationGaussianSphere::ConvertionPoseToGVector(tf::Quaternion q_pose)
{
	pcl::PointNormal g_vector;
	tf::Quaternion q_g_vector_global(0.0, 0.0, -1.0, 0.0);
	tf::Quaternion q_g_vector_local = q_pose.inverse()*q_g_vector_global*q_pose;
	g_vector.normal_x = q_g_vector_local.x();
	g_vector.normal_y = q_g_vector_local.y();
	g_vector.normal_z = q_g_vector_local.z();
	return g_vector;
}

void PoseEstimationGaussianSphere::CallbackInipose(const geometry_msgs::QuaternionConstPtr& msg)
{
	if(!inipose_is_available){
		inipose_is_available = true;
		tf::Quaternion q_inipose;
		quaternionMsgToTF(*msg, q_inipose);
		// q_inipose = tf::Quaternion(0.0, 0.0, 0.0, 1.0);	//test
		double r_calibration = atan2(rp_sincos_calibration[0][0], rp_sincos_calibration[0][1]);
		double p_calibration = atan2(rp_sincos_calibration[1][0], rp_sincos_calibration[1][1]);
		std::cout << "r_calibration = " << r_calibration << std::endl;
		std::cout << "p_calibration = " << p_calibration << std::endl;
		tf::Quaternion q_calibration = tf::createQuaternionFromRPY(r_calibration, p_calibration, 0.0)*q_inipose.inverse();
		lidar_alignment = Eigen::Quaternionf(q_calibration.w(), q_calibration.x(), q_calibration.y(), q_calibration.z());
		std::cout << "lidar_alignment = (" << lidar_alignment.x() << ", " << lidar_alignment.y() << ", " << lidar_alignment.z() << ", " << lidar_alignment.w() << ")"<< std::endl;
	}
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
	d_gaussian_sphere_newregistered_n->points.clear();
	list_num_dgauss_cluster_belongings.clear();
}

void PoseEstimationGaussianSphere::FittingWalls_::Compute(PoseEstimationGaussianSphere &mainclass, size_t i_start, size_t i_end)
{
	// std::cout << "NORMAL ESTIMATION" << std::endl;

	const size_t skip_step = 3;
	for(size_t i=i_start;i<i_end;i+=skip_step){
		bool input_to_gauss = true;
		bool input_to_dgauss = true;
		/*search neighbor points*/
		std::vector<int> indices;
		double laser_distance = sqrt(mainclass.cloud->points[i].x*mainclass.cloud->points[i].x + mainclass.cloud->points[i].y*mainclass.cloud->points[i].y + mainclass.cloud->points[i].z*mainclass.cloud->points[i].z);
		const double search_radius_min = 0.3;
		const double ratio = 0.09;
		double search_radius = ratio*laser_distance;
		if(search_radius<search_radius_min)	search_radius = search_radius_min;
		indices = mainclass.KdtreeSearch(mainclass.cloud->points[i], search_radius);
		/*judge*/
		// const size_t threshold_num_neighborpoints_gauss = 20;
		// const size_t threshold_num_neighborpoints_dgauss = 5;
		const size_t threshold_num_neighborpoints_gauss = 20;
		const size_t threshold_num_neighborpoints_dgauss = 20;
		if(indices.size()<threshold_num_neighborpoints_gauss)	input_to_gauss = false;
		if(indices.size()<threshold_num_neighborpoints_dgauss)	input_to_dgauss = false;
		if(!input_to_gauss && !input_to_dgauss)	continue;
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
		if(std::isnan(plane_parameters[0]) || std::isnan(plane_parameters[1]) || std::isnan(plane_parameters[2])){
			input_to_gauss = false;
			input_to_dgauss = false;
			continue;
		}
		/*judge*/
		const double threshold_angle = 30.0;	//[deg]
		if(fabs(fabs(mainclass.AngleBetweenVectors(tmp_normal, mainclass.g_vector_from_ekf))-M_PI/2.0)>threshold_angle/180.0*M_PI){
			input_to_gauss = false;
			input_to_dgauss = false;
			continue;
		}
		/*judge*/
		const double threshold_fitting_error = 0.01;	//[m]
		if(mainclass.ComputeSquareError(plane_parameters, indices)>threshold_fitting_error){
			input_to_gauss = false;
			input_to_dgauss = false;
			continue;
		}
		/*input*/
		normals_extracted_->points.push_back(tmp_normal);
		/*unit gaussian sphere*/
		pcl::PointXYZ tmp_point;
		if(input_to_gauss){
			tmp_point.x = plane_parameters[0];
			tmp_point.y = plane_parameters[1];
			tmp_point.z = plane_parameters[2];
			gaussian_sphere_->points.push_back(tmp_point);
		}
		/*d-gaussian sphere*/
		if(input_to_dgauss && !mainclass.mode_no_d_gauss){
			tmp_point.x = -plane_parameters[3]*plane_parameters[0];
			tmp_point.y = -plane_parameters[3]*plane_parameters[1];
			tmp_point.z = -plane_parameters[3]*plane_parameters[2];
			d_gaussian_sphere_->points.push_back(tmp_point);
		}
	}
}

void PoseEstimationGaussianSphere::FittingWalls_::Merge(PoseEstimationGaussianSphere &mainclass)
{
	*mainclass.normals += *normals_;
	*mainclass.normals_extracted += *normals_extracted_;
	*mainclass.gaussian_sphere += *gaussian_sphere_;
	*mainclass.d_gaussian_sphere += *d_gaussian_sphere_;
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
		// double square_error =	fabs(plane_parameters[0]*cloud->points[indices[i]].x
		// 							+plane_parameters[1]*cloud->points[indices[i]].y
		// 							+plane_parameters[2]*cloud->points[indices[i]].z
		// 							+plane_parameters[3])
		// 						/sqrt(plane_parameters[0]*plane_parameters[0]
		// 							+plane_parameters[1]*plane_parameters[1]
		// 							+plane_parameters[2]*plane_parameters[2]);
		// sum_square_error += square_error/(double)indices.size();
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
	// const int min_num_cluster_belongings = 70;
	const int min_num_cluster_belongings = 40;
	// const int min_num_cluster_belongings = 20;
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
		std::cout << ">> 0 normal" << std::endl;
		if(inipose_is_available)	cases_counter[0]++;
		return false;
	}
	else if(gaussian_sphere_clustered->points.size()==1){
		std::cout << ">> 1 normal" << std::endl;
		PartialRotation();
		rpy_cov_pub.data[3] = 5.0e+0;
		// rpy_cov_pub.data[3] = 5.0*ComputeLikelihood();
		if(inipose_is_available)	cases_counter[1]++;
	}
	else if(gaussian_sphere_clustered->points.size()==2){
		// std::cout << ">> 2 normals" << std::endl;
		/*judge*/
		const double threshold_angle = 20.0;	//[deg]
		if(fabs(AngleBetweenVectors(gaussian_sphere_clustered_n->points[0], gaussian_sphere_clustered_n->points[1]))<threshold_angle/180.0*M_PI){
			std::cout << "AngleBetweenVectors " << AngleBetweenVectors(gaussian_sphere_clustered_n->points[0], gaussian_sphere_clustered_n->points[1])/M_PI*180.0 << " > threshold_angle[deg]" << std::endl;
			if(inipose_is_available)	cases_counter[4]++;
			return false;
		}
		std::cout << "AngleBetweenVectors " << AngleBetweenVectors(gaussian_sphere_clustered_n->points[0], gaussian_sphere_clustered_n->points[1])/M_PI*180.0 << " [deg]" << std::endl;
		/*cross product*/
		g_vector_walls.normal_x = gaussian_sphere_clustered->points[0].y*gaussian_sphere_clustered->points[1].z - gaussian_sphere_clustered->points[0].z*gaussian_sphere_clustered->points[1].y;
		g_vector_walls.normal_y = gaussian_sphere_clustered->points[0].z*gaussian_sphere_clustered->points[1].x - gaussian_sphere_clustered->points[0].x*gaussian_sphere_clustered->points[1].z;
		g_vector_walls.normal_z = gaussian_sphere_clustered->points[0].x*gaussian_sphere_clustered->points[1].y - gaussian_sphere_clustered->points[0].y*gaussian_sphere_clustered->points[1].x;
		rpy_cov_pub.data[3] = 1.0e+0;
		// rpy_cov_pub.data[3] = 1.0e+1;
		// rpy_cov_pub.data[3] = 1.0e-1;
		// rpy_cov_pub.data[3] = ComputeLikelihood();
		if(inipose_is_available)	cases_counter[2]++;
	}
	else if(gaussian_sphere_clustered->points.size()>2){
		std::cout << ">> more than 3 normals" << std::endl;
		Eigen::Vector4f plane_parameters;
		float curvature;
		pcl::computePointNormal(*gaussian_sphere_clustered_weighted, plane_parameters, curvature);
		g_vector_walls.normal_x = plane_parameters[0];
		g_vector_walls.normal_y = plane_parameters[1];
		g_vector_walls.normal_z = plane_parameters[2];
		rpy_cov_pub.data[3] = 1.0e+0;
		// rpy_cov_pub.data[3] = 1.0e+1;
		// rpy_cov_pub.data[3] = 1.0e-1;
		// rpy_cov_pub.data[3] = ComputeLikelihood();
		if(inipose_is_available)	cases_counter[3]++;
	}
	/*flip*/
	flipNormalTowardsViewpoint(g_vector_walls, 0.0, 0.0, -100.0, g_vector_walls.normal_x, g_vector_walls.normal_y, g_vector_walls.normal_z);
	/*if angle variation is too large, estimation would be wrong*/
	// const double threshold_angle_variation = 30.0;	//[deg]
	const double threshold_angle_variation = 5.0;	//[deg]
	if(fabs(AngleBetweenVectors(g_vector_walls, g_vector_from_ekf))>threshold_angle_variation/180.0*M_PI){
		std::cout << ">> angle variation of g in 1 step is too large and would be wrong " << std::endl;
		if(inipose_is_available)	cases_counter[4]++;
		return false;
	}
	std::cout << "angle variation of g in 1 step: " << AngleBetweenVectors(g_vector_walls, g_vector_from_ekf)/M_PI*180.0 << " [deg]" << std::endl;
	// #<{(|normalization|)}>#
	// double norm_g = sqrt(g_vector_walls.normal_x*g_vector_walls.normal_x + g_vector_walls.normal_y*g_vector_walls.normal_y + g_vector_walls.normal_z*g_vector_walls.normal_z);
	// g_vector_walls.normal_x /= norm_g;
	// g_vector_walls.normal_y /= norm_g;
	// g_vector_walls.normal_z /= norm_g;
	// #<{(|convertion to roll, pitch|)}>#
	// rpy_cov_pub.data[0] = atan2(-g_vector_walls.normal_y, -g_vector_walls.normal_z);
	// rpy_cov_pub.data[1] = atan2(g_vector_walls.normal_x, sqrt(-g_vector_walls.normal_y*-g_vector_walls.normal_y + -g_vector_walls.normal_z*-g_vector_walls.normal_z));
	
	// q_rp_correction = GetRelativeRotationNormals(g_vector_from_ekf, g_vector_walls).inverse();

	return true;
}

double PoseEstimationGaussianSphere::ComputeLikelihood(void)
{
	double likelihood = 1.0;
	for(size_t i=0;i<gaussian_sphere_clustered_n->points.size();i++){
		double angle = AngleBetweenVectors(gaussian_sphere_clustered_n->points[i], g_vector_from_ekf);
		likelihood += fabs(cos(2*angle))/(double)gaussian_sphere_clustered_n->points.size();
		std::cout << "angle = " << angle << std::endl;
	}
	std::cout << "likelihood = " << likelihood << std::endl;
	return likelihood;
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

void PoseEstimationGaussianSphere::ClusterDGauss(void)
{
	// std::cout << "POINT CLUSTER" << std::endl;
	// const double cluster_distance = 0.3;
	const double cluster_distance = 0.5;
	// const int min_num_cluster_belongings = 20;
	const int min_num_cluster_belongings = 10;
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
		/*record number of belongings*/
		list_num_dgauss_cluster_belongings.push_back(cluster_indices[i].indices.size());
		/*for the Visualization*/
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
		/*for the visualization*/
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

	std::cout << "-------------------" << std::endl << "list_walls.size() = " << list_walls.size() << std::endl;
	if(list_walls.empty()){
		for(size_t i=0;i<d_gaussian_sphere_clustered->points.size();i++) InputNewWallInfo(d_gaussian_sphere_clustered->points[i]);
		return succeeded_y;
	}
	else{
		// const double ratio_matching_norm_dif = 0.2;
		// const double min_matching_norm_dif = 0.5;	//[m]
		// const double threshold_matching_norm_dif = 1.0;	//[m]
		// const double threshold_matching_angle = 15.0;	//[deg]
		const double threshold_matching_norm_dif = 0.5;	//[m]
		const double threshold_matching_angle = 40.0;	//[deg]
		// const int threshold_count_match = 5;
		const int threshold_count_match = 5;
		const int k = 1;
		kdtree.setInputCloud(d_gaussian_sphere_registered);
		for(size_t i=0;i<d_gaussian_sphere_clustered->points.size();i++){
			std::vector<int> pointIdxNKNSearch(k);
			std::vector<float> pointNKNSquaredDistance(k);
			if(kdtree.nearestKSearch(d_gaussian_sphere_clustered->points[i], k, pointIdxNKNSearch, pointNKNSquaredDistance)<=0)	std::cout << "kdtree error" << std::endl;
			double norm_clusterd = sqrt(d_gaussian_sphere_clustered->points[i].x*d_gaussian_sphere_clustered->points[i].x + d_gaussian_sphere_clustered->points[i].y*d_gaussian_sphere_clustered->points[i].y + d_gaussian_sphere_clustered->points[i].z*d_gaussian_sphere_clustered->points[i].z);
			double norm_registered = sqrt(d_gaussian_sphere_registered->points[pointIdxNKNSearch[0]].x*d_gaussian_sphere_registered->points[pointIdxNKNSearch[0]].x + d_gaussian_sphere_registered->points[pointIdxNKNSearch[0]].y*d_gaussian_sphere_registered->points[pointIdxNKNSearch[0]].y + d_gaussian_sphere_registered->points[pointIdxNKNSearch[0]].z*d_gaussian_sphere_registered->points[pointIdxNKNSearch[0]].z); 
			double angle = acos((d_gaussian_sphere_clustered->points[i].x*d_gaussian_sphere_registered->points[pointIdxNKNSearch[0]].x + d_gaussian_sphere_clustered->points[i].y*d_gaussian_sphere_registered->points[pointIdxNKNSearch[0]].y + d_gaussian_sphere_clustered->points[i].z*d_gaussian_sphere_registered->points[pointIdxNKNSearch[0]].z)/norm_clusterd/norm_registered);
			// double threshold_matching_norm_dif = ratio_matching_norm_dif*norm_clusterd;
			// if(threshold_matching_norm_dif<min_matching_norm_dif)	threshold_matching_norm_dif = min_matching_norm_dif;
			std::cout << "fabs(norm_clusterd-norm_registered) = |" << norm_clusterd << " - " << norm_registered << "| = " << fabs(norm_clusterd-norm_registered) << std::endl;
			std::cout << "fabs(angle/M_PI*180.0) = " << fabs(angle/M_PI*180.0) << std::endl;
			if(std::isnan(angle))	angle = 0.0;
			if(fabs(norm_clusterd-norm_registered)<threshold_matching_norm_dif && fabs(angle/M_PI*180.0)<threshold_matching_angle && !list_walls[pointIdxNKNSearch[0]].found_match){
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
							local_pose_error_rpy_sincosatan[j][0] += list_num_dgauss_cluster_belongings[i]*sin(tmp_local_pose_error_rpy[j]);
							local_pose_error_rpy_sincosatan[j][1] += list_num_dgauss_cluster_belongings[i]*cos(tmp_local_pose_error_rpy[j]);
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
			else{
				InputNewWallInfo(d_gaussian_sphere_clustered->points[i]);
				/*for the visualization*/
				pcl::PointNormal tmp_normal;
				tmp_normal.x = 0.0;
				tmp_normal.y = 0.0;
				tmp_normal.z = 0.0;
				tmp_normal.normal_x = d_gaussian_sphere_clustered->points[i].x;
				tmp_normal.normal_y = d_gaussian_sphere_clustered->points[i].y;
				tmp_normal.normal_z = d_gaussian_sphere_clustered->points[i].z;
				d_gaussian_sphere_newregistered_n->points.push_back(tmp_normal);
			}
		}
		/*arrange list*/
		const int threshold_count_nomatch = 5;
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
			if(compute_local_pose_error_in_quaternion)	q_ave_local_pose_error.normalize();
			else{
				for(int j=0;j<3;j++)	local_pose_error_rpy_sincosatan[j][2] = atan2(local_pose_error_rpy_sincosatan[j][0], local_pose_error_rpy_sincosatan[j][1]);
				q_ave_local_pose_error = tf::createQuaternionFromRPY(local_pose_error_rpy_sincosatan[0][2], local_pose_error_rpy_sincosatan[1][2], local_pose_error_rpy_sincosatan[2][2]);
			}
			tf::Quaternion q_pose_odom_now;
			quaternionMsgToTF(odom_now.pose.pose.orientation, q_pose_odom_now);
			quaternionTFToMsg(q_pose_odom_now*q_ave_local_pose_error, pose_pub.pose.orientation);
			std::cout << "succeeded matching" << std::endl;

			double rpy_last[3];
			double rpy_new[3];
			tf::Matrix3x3(q_pose_odom_now).getRPY(rpy_last[0], rpy_last[1], rpy_last[2]);
			tf::Matrix3x3(q_pose_odom_now*q_ave_local_pose_error).getRPY(rpy_new[0], rpy_new[1], rpy_new[2]);
			q_y_correction = tf::createQuaternionFromRPY(0.0, 0.0, atan2(sin(rpy_new[2] - rpy_last[2]), cos(rpy_new[2] - rpy_last[2])));
			q_y_correction = q_ave_local_pose_error;
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
	const double sigma_obs = 5.0e-1*wall.count_match;
	// const double sigma_obs = 1.0e+10;
	std::cout << "sigma_obs = " << sigma_obs << std::endl;
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
	if(std::isnan(theta)){
		std::cout << "theta is NAN" << std::endl;
		exit(1);
	}

	// std::cout << "local error: " << rpy[0] << ", " << rpy[1] << ", " << rpy[2] << std::endl;
	// std::cout << "distance: " << (Target - Origin).norm() << std::endl;
	// std::cout << "angle: " << acos(Origin.dot(Target)/Origin.norm()/Target.norm())/M_PI*180.0 << std::endl;
	// std::cout << "Origin: (" << Origin(0) << ", " << Origin(1) << ", " << Origin(2) << "), depth = " << Origin.norm() << std::endl;
	// std::cout << "Target: (" << Target(0) << ", " << Target(1) << ", " << Target(2) << "), depth = " << Target.norm() << std::endl;

	return relative_rotation;
}

void PoseEstimationGaussianSphere::FinalEstimation(bool succeeded_rp, bool succeeded_y)
{
	tf::Quaternion q_pose_odom_now;
	quaternionMsgToTF(odom_now.pose.pose.orientation, q_pose_odom_now);
	tf::Quaternion q_pose_new = q_pose_odom_now;
	if(succeeded_y)	q_pose_new = q_pose_new*q_y_correction;
	if(succeeded_rp){
		q_rp_correction = GetRelativeRotationNormals(ConvertionPoseToGVector(q_pose_new), g_vector_walls).inverse();
		q_pose_new = q_pose_new*q_rp_correction;
	}
	tf::Matrix3x3(q_pose_new).getRPY(rpy_cov_pub.data[0], rpy_cov_pub.data[1], rpy_cov_pub.data[2]);
	if(!succeeded_rp){
		rpy_cov_pub.data[0] = NAN;
		rpy_cov_pub.data[1] = NAN;
	}
	else if(!inipose_is_available){
		rp_sincos_calibration[0][0] += sin(rpy_cov_pub.data[0]);
		rp_sincos_calibration[0][1] += cos(rpy_cov_pub.data[0]);
		rp_sincos_calibration[1][0] += sin(rpy_cov_pub.data[1]);
		rp_sincos_calibration[1][1] += cos(rpy_cov_pub.data[1]);
	}
	if(!succeeded_y)	rpy_cov_pub.data[2] = NAN;
	// if(!succeeded_y && !succeeded_rp)	rpy_cov_pub.data[2] = NAN;
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

	viewer.addPointCloud(d_gaussian_sphere, "d_gaussian_sphere");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.8, "d_gaussian_sphere");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "d_gaussian_sphere");
	
	viewer.addPointCloudNormals<pcl::PointNormal>(d_gaussian_sphere_clustered_n, 1, 1.0, "d_gaussian_sphere_clustered_n");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.8, 0.0, 1.0, "d_gaussian_sphere_clustered_n");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1, "d_gaussian_sphere_clustered_n");
	
	viewer.addPointCloudNormals<pcl::PointNormal>(d_gaussian_sphere_registered_n, 1, 1.0, "d_gaussian_sphere_registered_n");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.8, 0.0, "d_gaussian_sphere_registered_n");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1, "d_gaussian_sphere_registered_n");

	viewer.addPointCloudNormals<pcl::PointNormal>(d_gaussian_sphere_newregistered_n, 1, 1.0, "d_gaussian_sphere_newregistered_n");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 0.0, "d_gaussian_sphere_newregistered_n");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "d_gaussian_sphere_newregistered_n");

	pcl::PointCloud<pcl::PointNormal>::Ptr tmp_g_vector_walls (new pcl::PointCloud<pcl::PointNormal>);
	tmp_g_vector_walls->points.push_back(g_vector_walls);
	viewer.addPointCloudNormals<pcl::PointNormal>(tmp_g_vector_walls, 1, 1.0, "g_vector_walls");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.5, 0.0, "g_vector_walls");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, "g_vector_walls");
	
	viewer.spinOnce();
}

void PoseEstimationGaussianSphere::Publication(void)
{
	pub_rpy.publish(rpy_cov_pub);

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
