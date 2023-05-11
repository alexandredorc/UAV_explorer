#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>

#include <octomap/octomap.h>


ros::Publisher pub;
bool callbackInProgress = false;

void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg) {
    // Convert OctoMap message to octree
    //ROS_INFO("test callback");

    if (callbackInProgress) {
        return;
    }

    callbackInProgress = true;

    octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(octomap_msgs::fullMsgToMap(*msg));
    
    // Convert octree to occupancy grid
    double resolution = octree->getResolution();
    double min_x, min_y, min_z;
    double max_x, max_y, max_z;
    octree->getMetricMin(min_x, min_y, min_z);
    octree->getMetricMax(max_x, max_y, max_z);
    
    
    int width = (max_x - min_x) / resolution;
    int height = (max_y - min_y) / resolution;
    int depth = (max_z - min_z) / resolution;
    //ROS_WARN("Size of occupancy grid data: %d %d %d", width,height,depth);
    nav_msgs::OccupancyGrid occupancy_grid_msg;
    occupancy_grid_msg.header.frame_id = "map";
    occupancy_grid_msg.info.resolution = resolution;
    occupancy_grid_msg.info.width = width;
    occupancy_grid_msg.info.height = height;
    occupancy_grid_msg.info.origin.position.x = -min_x/ resolution;
    occupancy_grid_msg.info.origin.position.y = -min_y/ resolution;
    occupancy_grid_msg.info.origin.position.z = -min_z/ resolution;

    occupancy_grid_msg.data.resize(width * height * depth);

    //ROS_WARN("Size of occupancy grid data: %d", occupancy_grid_msg.data.size());

    for (int z = 0; z < depth; z++) {
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
            
                double wx, wy, wz;
                octree->getMetricSize(wx, wy, wz);
                wx = (wx / width) * (x + 0.5) + min_x;
                wy = (wy / height) * (y + 0.5) + min_y;
                wz = (wz / depth) * (z + 0.5) + min_z;
                octomap::OcTreeNode* node = octree->search(wx, wy, wz);
                //ROS_WARN("test %d %d %d",x,y,z);
                if (node == NULL) {
                    // Cell is unknown
                    occupancy_grid_msg.data[(z * height * width) + (y * width) + x] = 0;
                } else if (octree->isNodeOccupied(node)) {
                    // Cell is occupied
                    occupancy_grid_msg.data[(z * height * width) + (y * width) + x] = 1;
                } else {
                    // Cell is free
                    occupancy_grid_msg.data[(z * height * width) + (y * width) + x] = -1;
                }
            }
        }
    }

    // Publish message to topic
    pub.publish(occupancy_grid_msg);

    callbackInProgress = false;

    delete octree;
}



int main(int argc, char** argv) {
    // Initialize ROS node
    ros::init(argc, argv, "octomap_to_occupancy_grid");
    
    ros::NodeHandle nh;

    // Subscribe to the octomap binary topic
    ros::Subscriber sub = nh.subscribe<octomap_msgs::Octomap>("octomap_full", 1, octomapCallback);
    
    // Publish the occupancy grid to a topic
    pub = nh.advertise<nav_msgs::OccupancyGrid>("occupancy_grid", 1);
    
    ros::Rate loop_rate(1);

    while (ros::ok()) {
        //ROS_INFO("loop test");
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
