#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
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
			bool fixed;
			bool found_match;
			int count_match;
			int count_nomatch;
		};
		/*pcl*/
		pcl::visualization::PCLVisualizer viewer{"d-gaussian sphere"};
		pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
		pcl::PointNormal g_vector_from_ekf;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud {new pcl::PointCloud<pcl::PointXYZ>};
		pcl::PointCloud<pcl::PointNormal>::Ptr normals {new pcl::PointCloud<pcl::PointNormal>};
		pcl::PointCloud<pcl::PointXYZ>::Ptr d_gauss {new pcl::PointCloud<pcl::PointXYZ>};
		pcl::PointCloud<pcl::PointXYZ>::Ptr centroids_now {new pcl::PointCloud<pcl::PointXYZ>};
		pcl::PointCloud<pcl::PointXYZ>::Ptr centroids_registered {new pcl::PointCloud<pcl::PointXYZ>};
		/*objects*/
		std::vector<WallInfo> list_walls;
		nav_msgs::Odometry odom_now;
		geometry_msgs::PoseStamped pose_pub;
		ros::Time time_pub;
		/*flags*/
		bool first_callback_odom = true;
	public:
		YawEstimationWalls();
		void CallbackPC(const sensor_msgs::PointCloud2ConstPtr &msg);
		void CallbackOdom(const nav_msgs::OdometryConstPtr& msg);
		void ClearCloud(void);
		void NormalEstimation(void);
		std::vector<int> KdtreeSearch(pcl::PointXYZ searchpoint, double search_radius);
		double AngleBetweenVectors(pcl::PointNormal v1, pcl::PointNormal v2);
		double ComputeSquareError(Eigen::Vector4f plane_parameters, std::vector<int> indices);
		void PointCluster(void);
		void CreateRegisteredCentroidCloud(void);
		pcl::PointXYZ PointTransformation(pcl::PointXYZ p, nav_msgs::Odometry origin, nav_msgs::Odometry target);
		bool MatchWalls(void);
		void InputNewWallInfo(pcl::PointXYZ p);
		void KalmanFilterForRegistration(WallInfo& wall);
		tf::Quaternion GetRelativeRotation(pcl::PointXYZ orgin, pcl::PointXYZ target);
		void Visualization(void);
		void Publication(void);
};

YawEstimationWalls::YawEstimationWalls()
{
	sub_pc = nh.subscribe("/velodyne_points", 1, &YawEstimationWalls::CallbackPC, this);
	sub_odom = nh.subscribe("/gyrodometry", 1, &YawEstimationWalls::CallbackOdom, this);
	pub_pose = nh.advertise<geometry_msgs::PoseStamped>("/pose", 1);
	viewer.setBackgroundColor(1, 1, 1);
	viewer.addCoordinateSystem(0.5, "axis");
	g_vector_from_ekf.x = 0.0;
	g_vector_from_ekf.y = 0.0;
	g_vector_from_ekf.z = 0.0;
	g_vector_from_ekf.normal_x = 0.0;
	g_vector_from_ekf.normal_y = 0.0;
	g_vector_from_ekf.normal_z = -1.0;
}

void YawEstimationWalls::CallbackPC(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	pcl::fromROSMsg(*msg, *cloud);
	time_pub = msg->header.stamp;
	ClearCloud();
	NormalEstimation();
	PointCluster();
	if(!first_callback_odom){
		CreateRegisteredCentroidCloud();
		bool succeeded_matching = MatchWalls();
		if(succeeded_matching)	Publication();
	}
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
	centroids_now->points.clear();
}

void YawEstimationWalls::NormalEstimation(void)
{
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
			// std::cout << ">> indices.size() = " << indices.size() << " < " << num_neighborpoints << ", then skip" << std::endl;
			continue;
		}
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
		/*judge*/
		const double threshold_angle = 30.0;	//[deg]
		if(fabs(AngleBetweenVectors(tmp_normal, g_vector_from_ekf)-M_PI/2.0)>threshold_angle/180.0*M_PI){
			// std::cout << ">> angle from square angle = " << fabs(AngleBetweenVectors(tmp_normal, g_vector_from_ekf->points[0])-M_PI/2.0) << " > " << threshold_angle << ", then skip" << std::endl;
			continue;
		}
		/*judge*/
		const double threshold_square_error = 0.01;
		if(ComputeSquareError(plane_parameters, indices)>threshold_square_error){
			// std::cout << ">> square error = " << ComputeSquareError(plane_parameters, indices) << " > " << threshold_square_error << ", then skip" << std::endl;
			continue;
		}
		/*delete nan*/
		if(std::isnan(plane_parameters[0]) || std::isnan(plane_parameters[1]) || std::isnan(plane_parameters[2])){
			// std::cout << ">> this point has NAN, then skip" << std::endl;
			continue;
		}
		/*input*/
		// std::cout << ">> ok, then input" << std::endl;
		pcl::PointXYZ tmp_point;
		tmp_point.x = -plane_parameters[3]*plane_parameters[0];
		tmp_point.y = -plane_parameters[3]*plane_parameters[1];
		tmp_point.z = -plane_parameters[3]*plane_parameters[2];
		d_gauss->points.push_back(tmp_point);
		
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

double YawEstimationWalls::AngleBetweenVectors(pcl::PointNormal v1, pcl::PointNormal v2)
{
	double dot_product = v1.normal_x*v2.normal_x + v1.normal_y*v2.normal_y + v1.normal_z*v2.normal_z;
	double v1_norm = sqrt(v1.normal_x*v1.normal_x + v1.normal_y*v1.normal_y + v1.normal_z*v1.normal_z);
	double v2_norm = sqrt(v2.normal_x*v2.normal_x + v2.normal_y*v2.normal_y + v2.normal_z*v2.normal_z);
	double angle = acosf(dot_product/(v1_norm*v2_norm));
	return angle;
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
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(d_gauss);
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ece;
	ece.setClusterTolerance (0.1);	//[m]
	ece.setMinClusterSize (10);
	ece.setMaxClusterSize (d_gauss->points.size());
	ece.setSearchMethod(tree);
	ece.setInputCloud(d_gauss);
	ece.extract(cluster_indices);

	std::cout << "cluster_indices.size() = " << cluster_indices.size() << std::endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_clustered_points (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointIndices::Ptr tmp_clustered_indices (new pcl::PointIndices);
	for(size_t i=0;i<cluster_indices.size();i++){
		/*extract*/
		*tmp_clustered_indices = cluster_indices[i];
		pcl::ExtractIndices<pcl::PointXYZ> ei;
		ei.setInputCloud(d_gauss);
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
		centroids_now->points.push_back(tmp_centroid);
	}
}

void YawEstimationWalls::CreateRegisteredCentroidCloud(void)
{
	for(size_t i=0;i<list_walls.size();i++){
		centroids_registered->points.push_back(PointTransformation(list_walls[i].point, list_walls[i].odom, odom_now));
	}
}

pcl::PointXYZ YawEstimationWalls::PointTransformation(pcl::PointXYZ p, nav_msgs::Odometry origin, nav_msgs::Odometry target)
{
	/*linear*/
	double delta_x = target.pose.pose.position.x - origin.pose.pose.position.x;
	double delta_y = target.pose.pose.position.y - origin.pose.pose.position.y;
	double delta_z = target.pose.pose.position.z - origin.pose.pose.position.z;
	/*rotation*/
	tf::Quaternion q_point_origin(p.x, p.y, p.z, 1.0);
	tf::Quaternion q_pose_origin;
	tf::Quaternion q_pose_target;
	quaternionMsgToTF(origin.pose.pose.orientation, q_pose_origin);
	quaternionMsgToTF(target.pose.pose.orientation, q_pose_target);
	tf::Quaternion relative_rotation = q_pose_target*q_pose_origin.inverse();
	relative_rotation.normalize();
	tf::Quaternion q_point_target = relative_rotation*q_point_origin;
	/*input*/
	pcl::PointXYZ p_;
	p_.x = q_point_target.x() + delta_x;
	p_.y = q_point_target.y() + delta_y;
	p_.z = q_point_target.z() + delta_z;
	return p_;
}

bool YawEstimationWalls::MatchWalls(void)
{
	const double threshold_matching_distance = 0.1;
	const int threshold_count_match = 5;
	const int k = 1;
	kdtree.setInputCloud(centroids_registered);
	std::vector<tf::Quaternion> list_local_pose_error;
	for(size_t i=0;i<centroids_now->points.size();i++){
		std::vector<int> pointIdxNKNSearch(k);
		std::vector<float> pointNKNSquaredDistance(k);
		if(kdtree.nearestKSearch(centroids_now->points[i], k, pointIdxNKNSearch, pointNKNSquaredDistance)<=0)	std::cout << "kdtree error" << std::endl;
		if(sqrt(pointNKNSquaredDistance[0])<threshold_matching_distance){
			list_walls[pointIdxNKNSearch[0]].found_match = true;
			list_walls[pointIdxNKNSearch[0]].count_match++;
			if(list_walls[pointIdxNKNSearch[0]].fixed){
				list_local_pose_error.push_back(GetRelativeRotation(list_walls[pointIdxNKNSearch[0]].point, centroids_now->points[i]));
			}
			else{
				list_walls[pointIdxNKNSearch[0]].point = centroids_now->points[i];
				KalmanFilterForRegistration(list_walls[pointIdxNKNSearch[0]]);
				if(list_walls[pointIdxNKNSearch[0]].count_match>threshold_count_match){
					list_walls[pointIdxNKNSearch[0]].point.x = list_walls[pointIdxNKNSearch[0]].X(0, 0);
					list_walls[pointIdxNKNSearch[0]].point.y = list_walls[pointIdxNKNSearch[0]].X(1, 0);
					list_walls[pointIdxNKNSearch[0]].point.z = list_walls[pointIdxNKNSearch[0]].X(2, 0);
					list_walls[pointIdxNKNSearch[0]].fixed = true;
				}
			}
		}
		else	InputNewWallInfo(centroids_now->points[i]);
	}
	const int threshold_count_nomatch = 5;
	for(size_t i=0;i<list_walls.size();i++){
		if(!list_walls[i].found_match)	list_walls[i].count_nomatch++;
		if(list_walls[i].count_nomatch>threshold_count_nomatch){
			list_walls.erase(list_walls.begin() + i);
			i--;
		}
		else	list_walls[i].found_match = false;
	}
	if(!list_local_pose_error.empty()){
		tf::Quaternion ave_local_pose_error = list_local_pose_error[0];
		for(size_t i=1;i<list_local_pose_error.size();i++) ave_local_pose_error += list_local_pose_error[i];
		ave_local_pose_error.normalize();
		tf::Quaternion q_pose_odom_now;
		quaternionMsgToTF(odom_now.pose.pose.orientation, q_pose_odom_now);
		quaternionTFToMsg(q_pose_odom_now*ave_local_pose_error, pose_pub.pose.orientation);
		return true;
	}
	else	return false;
}

void YawEstimationWalls::InputNewWallInfo(pcl::PointXYZ p)
{
	WallInfo tmp_wallinfo;
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

void YawEstimationWalls::KalmanFilterForRegistration(WallInfo& wall)
{
	const int num_state = 3;
	/*prediction*/
	Eigen::MatrixXd A = Eigen::MatrixXd::Identity(num_state, num_state);
	Eigen::MatrixXd F(num_state, num_state);
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
	const double sigma_obs = 1.0e+1;
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
}

tf::Quaternion YawEstimationWalls::GetRelativeRotation(pcl::PointXYZ origin, pcl::PointXYZ target)
{
	tf::Quaternion q_point_origin(origin.x, origin.y, origin.z, 1.0);
	tf::Quaternion q_point_target(target.x, target.y, target.z, 1.0);
	return q_point_target*q_point_origin.inverse();
}

void YawEstimationWalls::Visualization(void)
{
	viewer.removeAllPointClouds();

	viewer.addPointCloud(cloud, "cloud");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 0.0, "cloud");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
	
	viewer.addPointCloudNormals<pcl::PointNormal>(normals, 1, 0.5, "normals");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 1.0, "normals");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1, "normals");
	
	viewer.addPointCloud(d_gauss, "d_gauss");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "d_gauss");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "d_gauss");
	
	viewer.addPointCloud(centroids_now, "centroids_now");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 0.0, "centroids_now");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "centroids_now");
	
	viewer.spinOnce();
}

void YawEstimationWalls::Publication(void)
{
	pose_pub.header.frame_id = odom_now.header.frame_id;
	pose_pub.header.stamp = time_pub;
	pub_pose.publish(pose_pub);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "yaw_estimation_walls");
	
	YawEstimationWalls yaw_estimation_walls;

	ros::spin();
}
