#include <condition_variable>
#include <queue>
#include <thread>
#include <math.h>
#include <dirent.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/Bool.h>
#include <tf/transform_listener.h>

#include <Eigen/Core>

namespace {

struct tile {
	std::string path;
	float origin_x;
	float origin_y;
	int id;
	std::vector<tile*> neighbors;
};

bool checkNeighbor(tile i, tile j, float thresh)
{
	float dist = abs(i.origin_x - j.origin_x) + abs(i.origin_y - j.origin_y);
	return dist <= thresh;
}

class ReflectivityMapLoader {
private:
	ros::NodeHandle& nodeHandle_;
	ros::Publisher pcdPublisher_;
	ros::Subscriber poseSubscriber_;
	Eigen::Vector3d pose_;
	pcl::PointCloud<pcl::PointXYZRGB> cloudFull_;
	std::string pcdPublishTopic_;
	std::string poseSubscribeTopic_;
	double visibilityRadius_;
	std::vector<tile> allTiles_;
	bool poseAvailable_;
	float neighborDist_;
	float publishRate_;

public:
	ReflectivityMapLoader(ros::NodeHandle &nodeHandle);
	~ReflectivityMapLoader();
	void readParameters();
	void createPcd(const std::vector<std::string>& pcdPaths);
	void publishPcd (sensor_msgs::PointCloud2 &cloud);
	void poseCallback(const geometry_msgs::PoseStamped &msg);
	void getPoseTile();
	double computeDist(Eigen::Vector3d p, Eigen::Vector3d q);
	void createTiles(std::vector<std::string> pcdPaths);
	void getTileCenters(std::string path, float &x, float &y);
	void computeNeighbors();
	bool onTile(Eigen::Vector3d pose, tile t);
	void managePcdPublish(tile t);
	void getPublishRate(float &rate);
};

ReflectivityMapLoader::ReflectivityMapLoader(ros::NodeHandle &nodeHandle)
	:nodeHandle_(nodeHandle),
	poseAvailable_(0)
{
	readParameters();
	poseSubscriber_ = nodeHandle_.subscribe(poseSubscribeTopic_, 500, &ReflectivityMapLoader::poseCallback, this);
	pcdPublisher_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>(pcdPublishTopic_, publishRate_);
}

ReflectivityMapLoader::~ReflectivityMapLoader()
{

}

void ReflectivityMapLoader::readParameters()
{
  nodeHandle_.param("pcd_topic", pcdPublishTopic_, std::string("/reflectivity_map"));
  nodeHandle_.param("pose_topic", poseSubscribeTopic_, std::string("/pose_ground_truth"));
	nodeHandle_.param("neighbor_dist", neighborDist_, (float)192.0); // factor of 64
	nodeHandle_.param("publish_rate", publishRate_, (float)1.0); // factor of 64
}

void ReflectivityMapLoader::poseCallback(const geometry_msgs::PoseStamped &msg)
{
	pose_(0) = msg.pose.position.x;
	pose_(1) = msg.pose.position.y;
	pose_(2) = msg.pose.position.z;
	poseAvailable_ = 1;
}

void ReflectivityMapLoader::createPcd(const std::vector<std::string>& pcdPaths)
{
	sensor_msgs::PointCloud2 pcdFull, pcdPart;
	for (const std::string& path : pcdPaths) {
		if (pcdFull.width == 0) {
			if (pcl::io::loadPCDFile(path.c_str(), pcdFull) == -1) {
			}
		} else {
			if (pcl::io::loadPCDFile(path.c_str(), pcdPart) == -1) {
			}
			pcdFull.width += pcdPart.width;
			pcdFull.row_step += pcdPart.row_step;
			pcdFull.data.insert(pcdFull.data.end(), pcdPart.data.begin(), pcdPart.data.end());
		}
		if (!ros::ok()) break;
	}

	publishPcd(pcdFull);
}

void ReflectivityMapLoader::getPoseTile()
{

	// find which tile is the vehicle in
	if (poseAvailable_)
	{
		Eigen::Vector3d currentPose(pose_);
		for (std::vector<tile>::iterator i = allTiles_.begin(); i != allTiles_.end(); i++)
		{
			bool isInside = onTile(currentPose, *i);
			if (isInside)
			{
				managePcdPublish(*i);
				return;
			}
		}
	}
	ROS_INFO("Waiting for pose");
	return;
}

void ReflectivityMapLoader::managePcdPublish(tile t)
{
	std::vector<std::string> toBeLoaded;
	std::vector<int> currentlyLoaded_;
	// Load the neighbors
	for (std::vector<tile*>::iterator i = t.neighbors.begin(); i != t.neighbors.end(); i++)
	{
		currentlyLoaded_.push_back((*i)->id);
		toBeLoaded.push_back((*i)->path);
	}

	ROS_INFO("Loading %d tiles",(int)toBeLoaded.size());
	createPcd(toBeLoaded);
}

void ReflectivityMapLoader::publishPcd(sensor_msgs::PointCloud2 &pcd)
{
	if (pcd.width != 0) {
		pcd.header.frame_id = "map";
		pcdPublisher_.publish(pcd);
	}
}

double ReflectivityMapLoader::computeDist(Eigen::Vector3d p, Eigen::Vector3d q)
{
	double dist = sqrt(pow(p(0) - q(0), 2) +
										 pow(p(1) - q(1), 2));
	return dist;
}

void ReflectivityMapLoader::getTileCenters(std::string path, float &x, float &y)
{
	std::string splitPath;
	std::vector<size_t> vec;
	size_t pos = path.find('/');
	while (pos != std::string::npos)
	{
		vec.push_back(pos);
		pos = path.find('/', pos + 1);
	}
	pos = vec.back();
	std::string filename = path.substr(pos + 1, path.find('.'));
	pos = filename.find('_');
	size_t pos_end = filename.find('.');
	std::string x1 = filename.substr(0,pos);
	std::string y1 = filename.substr(pos+1,pos_end - pos -1);
	x = std::stof(x1);
	y = std::stof(y1);
}

void ReflectivityMapLoader::createTiles(std::vector<std::string> pcdPaths)
{
	int id_counter = 0;
	for (std::vector<std::string>::iterator i = pcdPaths.begin(); i != pcdPaths.end(); i++)
	{
		float x,y;
		getTileCenters(*i, x, y);
		tile newTile;
		newTile.origin_x = x;
		newTile.origin_y = y;
		newTile.path = *i;
		newTile.id = id_counter;
		allTiles_.push_back(newTile);
		id_counter++;
	}
	ROS_INFO("Total number of tiles added are %d",(int)allTiles_.size());
}

void ReflectivityMapLoader::computeNeighbors()
{
	for (std::vector<tile>::iterator i = allTiles_.begin(); i != allTiles_.end(); i++)
	{
		for (std::vector<tile>::iterator j = allTiles_.begin(); j != allTiles_.end(); j++)
		{
			bool isNeighbor = checkNeighbor(*i, *j, neighborDist_);
			if (isNeighbor)
			{
				i->neighbors.push_back(&(*j));
			}

		}
	}
}

bool ReflectivityMapLoader::onTile(Eigen::Vector3d pose, tile t)
{
	float pos_x = (float)pose(0);
	float pos_y = (float)pose(1);
	return (pos_x > t.origin_x && pos_y > t.origin_y &&
					pos_x < t.origin_x + 64.0 && pos_y < t.origin_y + 64);
}

bool getFileNames(std::string folderPath, std::vector<std::string> &pcdPaths)
{
	DIR* dirp = opendir(folderPath.c_str());
	struct dirent * dp;
	if (dirp != NULL) {
		while ((dp = readdir(dirp)) != NULL) {

			// only add pcd files
			std::size_t type = std::string(dp->d_name).find(".pcd");
			if (type != std::string::npos) {
				pcdPaths.push_back(folderPath + std::string(dp->d_name));
			}
		}
		closedir(dirp);
	} else {
		/* could not open directory */
  	perror ("Cannot open dir");
  	return EXIT_FAILURE;
	}
	return EXIT_SUCCESS;
}

void ReflectivityMapLoader::getPublishRate(float &rate)
{
	rate = publishRate_;
	return;
}

void print_usage()
{
	ROS_ERROR_STREAM("Usage:");
	ROS_ERROR_STREAM("rosrun map_loader reflectivity_map_loader reflectivity_map_folder");
}

} // namespace

int main(int argc, char **argv)
{
	ros::init(argc, argv, "reflectivity_map_folder");
	ros::NodeHandle nh("~");
	if (argc < 2 || (argc == 2 && std::string(argv[1]) == "-h")) {
		print_usage();
		return EXIT_FAILURE;
	}

	std::string folderPath("");
	if (argc == 2){
    folderPath = std::string(argv[1]);
  }

	ReflectivityMapLoader reflectivityMapLoader(nh);
	std::vector<std::string> pcdPaths;

	if (!getFileNames(folderPath, pcdPaths)) {
		reflectivityMapLoader.createTiles(pcdPaths);
		reflectivityMapLoader.computeNeighbors();

		float publishRate;
		reflectivityMapLoader.getPublishRate(publishRate);
		ros::Rate rate(publishRate);
		while (ros::ok())
		{
			reflectivityMapLoader.getPoseTile();
			ros::spinOnce();
			rate.sleep();
		}
	} else {
		return EXIT_FAILURE;
	}


	return 0;
}
