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
		pcl::PointCloud<pcl::InterestPoint>::Ptr walls_now {new pcl::PointCloud<pcl::InterestPoint>};
		pcl::PointCloud<pcl::InterestPoint>::Ptr walls_now_rotated {new pcl::PointCloud<pcl::InterestPoint>};
		pcl::PointCloud<pcl::InterestPoint>::Ptr walls_last {new pcl::PointCloud<pcl::InterestPoint>};
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
		double ComputeYawRate(pcl::InterestPoint p1, pcl::InterestPoint p2);
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
	// std::cout << "CALLBACK POSE" << std::endl;
	quaternionMsgToTF(msg->pose.orientation, pose_now);
	if(first_callback_pose)	pose_last = pose_now;
	
	first_callback_pose = false;
}

void YawEstimationWalls::CallbackNormals(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	// std::cout << "CALLBACK NORMALS" << std::endl;
	pcl::PointCloud<pcl::InterestPoint>::Ptr tmp_pc (new pcl::PointCloud<pcl::InterestPoint>);
	pcl::fromROSMsg(*msg, *tmp_pc);
	walls_now->points.clear();
	for(size_t i=1;i<tmp_pc->points.size();i++)	walls_now->points.push_back(tmp_pc->points[i]);

	if(!first_callback_pose)	MatchWalls();

	Visualizer();
	
	pose_last = pose_now;
	*walls_last = *walls_now;
}

void YawEstimationWalls::MatchWalls(void)
{
	// std::cout << "MATCh WALLS" << std::endl;
	if(walls_last->points.empty()){
		*walls_last = *walls_now;
		viewer.addSphere (walls_last->points[0], 1.0, 0.5, 0.5, 0.0, "sphere");
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
		pcl::KdTreeFLANN<pcl::InterestPoint> kdtree;
		std::vector<int> pointIdxNKNSearch(k);
		std::vector<float> pointNKNSquaredDistance(k);
		kdtree.setInputCloud(walls_now_rotated);
		const double threshold_matching_distance = 0.5;
		std::vector<double> list_yawrate;
		std::vector<double> list_strength;
		for(size_t i=0;i<walls_last->points.size();i++){
			if(kdtree.nearestKSearch(walls_last->points[i], k, pointIdxNKNSearch, pointNKNSquaredDistance)<=0)	std::cout << "kdtree error" << std::endl;
			if(sqrt(pointNKNSquaredDistance[0])<threshold_matching_distance){
				std::cout << "pointNKNSquaredDistance[0] = " << pointNKNSquaredDistance[0] << std::endl;
				std::cout << "sqrt(pointNKNSquaredDistance[0]) = " << sqrt(pointNKNSquaredDistance[0]) << std::endl;
				double yawrate = ComputeYawRate(walls_now->points[pointIdxNKNSearch[0]], walls_last->points[i]);
				list_yawrate.push_back(yawrate);
				list_strength.push_back(walls_now->points[pointIdxNKNSearch[0]].strength + walls_last->points[i].x);
			}
		}
		viewer.updateSphere(walls_last->points[0], threshold_matching_distance, 0.5, 0.5, 0.0, "sphere");
		if(!list_yawrate.empty()){
			double yawrate_ave = 0.0;
			double strength_sum = 0.0;
			for(size_t i=0;i<list_yawrate.size();i++){
				yawrate_ave += list_strength[i]*list_yawrate[i];
				strength_sum += list_strength[i];
			}
			yawrate_ave /= strength_sum;
			// std::cout << "yaw rate = " << yawrate_ave << std::endl;
			pub.publish(yawrate_ave);
		}
	}
}

double YawEstimationWalls::ComputeYawRate(pcl::InterestPoint p_origin, pcl::InterestPoint p_target)
{
	tf::Quaternion q1(p_origin.x, p_origin.y, p_origin.z, 1.0);
	tf::Quaternion q2(p_target.x, p_target.y, p_target.z, 1.0);
	tf::Quaternion relative_rotation = (pose_last*q2)*(pose_last*q1).inverse();
	relative_rotation.normalize();
	double roll, pitch, yaw;
	tf::Matrix3x3(relative_rotation).getRPY(roll, pitch, yaw);
	std::cout << "yaw rate = " << yaw << std::endl;
	std::cout << "acosf(q1.x()*q2.x()+q1.y()*q2.y()+q1.z()*q2.z()) = " << acosf(q1.x()*q2.x()+q1.y()*q2.y()+q1.z()*q2.z()) << std::endl;
	return yaw;
}

void YawEstimationWalls::Visualizer(void)
{
	// std::cout << "VISUALIZER" << std::endl;
	
	viewer.removePointCloud("walls_now");
	viewer.addPointCloud<pcl::InterestPoint>(walls_now, "walls_now");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "walls_now");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "walls_now");
	
	viewer.removePointCloud("walls_now_rotated");
	viewer.addPointCloud<pcl::InterestPoint>(walls_now_rotated, "walls_now_rotated");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "walls_now_rotated");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "walls_now_rotated");

	viewer.removePointCloud("walls_last");
	viewer.addPointCloud<pcl::InterestPoint>(walls_last, "walls_last");
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
