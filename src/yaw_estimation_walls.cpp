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
			Eigen::MatrixXd X{3, 1};
			Eigen::MatrixXd P{3, 3};
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
		void ClearPoints(void);
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
	// sub_odom = nh.subscribe("/gyrodometry", 1, &YawEstimationWalls::CallbackOdom, this);
	sub_odom = nh.subscribe("/combined_odometry", 1, &YawEstimationWalls::CallbackOdom, this);
	pub_pose = nh.advertise<geometry_msgs::PoseStamped>("/pose_dgauss", 1);
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
	// std::cout << "CALLBACK PC" << std::endl;
	pcl::fromROSMsg(*msg, *cloud);
	time_pub = msg->header.stamp;
	ClearPoints();
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
	// std::cout << "CALLBACK ODOM" << std::endl;
	odom_now = *msg;

	first_callback_odom = false;
}

void YawEstimationWalls::ClearPoints(void)
{
	// std::cout << "CLEAR POINTS" << std::endl;
	d_gauss->points.clear();
	normals->points.clear();
	centroids_now->points.clear();
	centroids_registered->points.clear();
}

void YawEstimationWalls::NormalEstimation(void)
{
	// std::cout << "NORMAL ESTIMATION" << std::endl;
	kdtree.setInputCloud(cloud);

	const size_t skip_step = 3;
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
		const size_t num_neighborpoints = 40;
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
	// std::cout << "POINT CLUSTER" << std::endl;
	const double threshold_cluster_distance = 0.15;	//[m]
	const int minimum_cluster_belongings = 25;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(d_gauss);
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ece;
	ece.setClusterTolerance (threshold_cluster_distance);
	ece.setMinClusterSize(minimum_cluster_belongings);
	ece.setMaxClusterSize(d_gauss->points.size());
	ece.setSearchMethod(tree);
	ece.setInputCloud(d_gauss);
	ece.extract(cluster_indices);

	// std::cout << "cluster_indices.size() = " << cluster_indices.size() << std::endl;

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
	/*rotation*/
	tf::Quaternion q_point_origin(p.x, p.y, p.z, 1.0);
	tf::Quaternion q_pose_origin;
	tf::Quaternion q_pose_target;
	quaternionMsgToTF(origin.pose.pose.orientation, q_pose_origin);
	quaternionMsgToTF(target.pose.pose.orientation, q_pose_target);
	tf::Quaternion relative_rotation = q_pose_origin*q_pose_target.inverse();
	relative_rotation.normalize();
	tf::Quaternion q_point_target = relative_rotation*q_point_origin*relative_rotation.inverse();
	/*linear*/
	tf::Quaternion q_global_move(
			target.pose.pose.position.x - origin.pose.pose.position.x,
			target.pose.pose.position.y - origin.pose.pose.position.y,
			target.pose.pose.position.z - origin.pose.pose.position.z,
			1.0);
	tf::Quaternion q_local_move = q_pose_origin.inverse()*q_global_move*q_pose_origin;
	Eigen::Vector3d Move(q_local_move.x(), q_local_move.y(), q_local_move.z());
	Eigen::Vector3d Normal(p.x, p.y, p.z);
	Move = (Move.dot(Normal)/Normal.dot(Normal))*Normal;
	/*input*/
	pcl::PointXYZ p_;
	p_.x = q_point_target.x() - Move(0);
	p_.y = q_point_target.y() - Move(1);
	p_.z = q_point_target.z() - Move(2);
	return p_;
}

bool YawEstimationWalls::MatchWalls(void)
{
	// std::cout << "MATCH WALLS" << std::endl;

	std::cout << "list_walls.size() = " << list_walls.size() << std::endl;
	if(list_walls.empty()){
		for(size_t i=0;i<centroids_now->points.size();i++) InputNewWallInfo(centroids_now->points[i]);
		return false;
	}
	else{
		const double threshold_matching_distance = 0.5;
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
				list_walls[pointIdxNKNSearch[0]].count_nomatch = 0;
				if(list_walls[pointIdxNKNSearch[0]].fixed){
					// list_local_pose_error.push_back(GetRelativeRotation(centroids_now->points[i], list_walls[pointIdxNKNSearch[0]].point));
					list_local_pose_error.push_back(GetRelativeRotation(centroids_now->points[i], centroids_registered->points[pointIdxNKNSearch[0]]));
				}
				else{
					list_walls[pointIdxNKNSearch[0]].point = centroids_now->points[i];
					KalmanFilterForRegistration(list_walls[pointIdxNKNSearch[0]]);
					if(list_walls[pointIdxNKNSearch[0]].count_match>threshold_count_match)	list_walls[pointIdxNKNSearch[0]].fixed = true;
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
			std::cout << "succeeded matching" << std::endl;
			std::cout << "list_local_pose_error.size() = " << list_local_pose_error.size() << std::endl;
			return true;
		}
		else	return false;
	}
}

void YawEstimationWalls::InputNewWallInfo(pcl::PointXYZ p)
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

void YawEstimationWalls::KalmanFilterForRegistration(WallInfo& wall)
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
	const double sigma_obs = 1.0e+100;
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
	
	std::cout << "Y = " << std::endl << Y << std::endl;
	std::cout << "K*Y = " << std::endl << K*Y << std::endl;
}

// tf::Quaternion YawEstimationWalls::GetRelativeRotation(pcl::PointXYZ origin, pcl::PointXYZ target)
// {
// 	Eigen::Vector3d Origin(origin.x, origin.y, origin.z);
// 	Eigen::Vector3d Target(target.x, target.y, target.z);
// 	double theta = acos(Origin.dot(Target)/Origin.norm()/Target.norm());
// 	Eigen::Vector3d Axis = Origin.cross(Target);
// 	Axis.normalize();
// 	tf::Quaternion relative_rotation(sin(theta/2.0)*Axis(0), sin(theta/2.0)*Axis(1), sin(theta/2.0)*Axis(2), cos(theta/2.0));
// 	relative_rotation.normalize();
// 	return relative_rotation;
// }
tf::Quaternion YawEstimationWalls::GetRelativeRotation(pcl::PointXYZ origin, pcl::PointXYZ target)
{
	tf::Quaternion q_point_origin(origin.x, origin.y, origin.z, 1.0);
	tf::Quaternion q_point_target(target.x, target.y, target.z, 1.0);
	q_point_origin.normalize();
	q_point_target.normalize();
	tf::Quaternion relative_rotation = q_point_target*q_point_origin.inverse();
	relative_rotation.normalize();
	return relative_rotation;
}

void YawEstimationWalls::Visualization(void)
{
	viewer.removeAllPointClouds();

	viewer.addPointCloud(cloud, "cloud");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 0.0, "cloud");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
	
	viewer.addPointCloudNormals<pcl::PointNormal>(normals, 1, 0.5, "normals");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 1.0, "normals");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, "normals");
	
	viewer.addPointCloud(d_gauss, "d_gauss");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "d_gauss");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "d_gauss");
	
	viewer.addPointCloud(centroids_now, "centroids_now");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 0.0, "centroids_now");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "centroids_now");
	
	viewer.addPointCloud(centroids_registered, "centroids_registered");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "centroids_registered");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "centroids_registered");
	
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
