#include <ros/ros.h>
#include <ros/node_handle.h>

#include <octomap_server/OctomapServer.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <message_filters/subscriber.h>

#include <sensor_msgs/PointCloud2.h>
#include "../include/octomap_server/extra_point_types.h"
#include "../include/octomap_server/pcl_conversions.h"

#define LEAF_SIZE (0.01f)

//#define VOXEL_FILTER

typedef pcl::PointXYZDGI PCLPoint1;
typedef pcl::PointXYZDGIP PCLPoint2;
typedef pcl::PointCloud<pcl::PointXYZDGI> MyPointCloud1;
typedef pcl::PointCloud<pcl::PointXYZDGIP> MyPointCloud2;

class DGIPServer {
public:
    //Constructor
    DGIPServer() {
        ROS_INFO("DGIP: IN SERVER\n");
        onInit();
    }

private:

    virtual void onInit() {

        ROS_INFO("DGIP: IN INIT\n");

        ros::NodeHandle nh; 

        std::string topic = "/dgi_points";

        // queue size
        int queue_size = 2;

        //Topic you want to publish
        pub_ = nh.advertise<sensor_msgs::PointCloud2>("dgip_points", 1000);

        ROS_INFO("DGIP: Calling callback\n");
        sub_= nh.subscribe<sensor_msgs::PointCloud2> (topic, queue_size, &DGIPServer::callback, this);

    }

    void filterOutput(const MyPointCloud2& pcl_unfiltered, sensor_msgs::PointCloud2& ros_filtered) {
        /*    
        pcl_unfiltered
          TO
        ros_unfiltered
          TO
        pc2_unfiltered
          TO (via filter)
        pc2_filtered
          TO
        ros_filtered
        */
        sensor_msgs::PointCloud2 ros_unfiltered;
        pcl::PCLPointCloud2::Ptr pc2_unfiltered_ptr(new pcl::PCLPointCloud2);
        pcl::PCLPointCloud2 pc2_filtered;

        pcl::VoxelGrid<pcl::PCLPointCloud2> sor;

        toROSMsg(pcl_unfiltered, ros_unfiltered);

        pcl_conversions::toPCL(ros_unfiltered, *pc2_unfiltered_ptr);

        sor.setInputCloud(pc2_unfiltered_ptr);

        sor.setLeafSize (LEAF_SIZE, LEAF_SIZE, LEAF_SIZE);

        sor.filter (pc2_filtered);

        pcl_conversions::fromPCL(pc2_filtered, ros_filtered);
    }

    PCLPoint2 get_DGIP_point(PCLPoint1 in) {
        float p;
        if ((float)in.i <= 0) {
            p = 0.0;
        } else if ((float)in.g >= (float)in.i) {
            p = 1.0;
        } else {
            p = ((float)in.g)/((float)in.i);
        }
        return PCLPoint2(in.x, in.y, in.z, in.d, in.g, in.i, p);
    }

    // callback signature
    void callback(const sensor_msgs::PointCloud2ConstPtr& ros_in) {
        
        MyPointCloud1 pcl_in;
        MyPointCloud2 pcl_out;
        sensor_msgs::PointCloud2 ros_out;

        // void fromROSMsg(const sensor_msgs::PointCloud2 &cloud, pcl::PointCloud<T> &pcl_cloud)

        //pcl::PCLPointCloud2 pcl_pc2;
        //pcl_conversions::toPCL(cloud, pcl_pc2);
        //pcl::fromPCLPointCloud2(pcl_pc2, pcl_cloud);
        fromROSMsg(*ros_in, pcl_in);

        pcl_out.header = pcl_in.header;

        pcl_out.height = pcl_in.height;
        pcl_out.width = pcl_in.width;

        int i, j;
        for (i=0; i < pcl_in.height; i++) {
            for (j=0; j < pcl_in.width; j++) {
                pcl_out.points.push_back(get_DGIP_point(pcl_in.points[i * pcl_in.width + j]));
            }
        }

#ifdef VOXEL_FILTER
        filterOutput(pcl_out, ros_out);
#else
        toROSMsg(pcl_out, ros_out);
#endif

        pub_.publish(ros_out);
    }

    ros::Subscriber sub_;
    ros::Publisher pub_;

};//End of class DGIPServer

int main(int argc, char **argv) {

    //Initiate ROS
    ros::init(argc, argv, "dgip_server_node");

    //Create an object of class DGIPServer that will take care of everything
    DGIPServer dgip_server;

    ros::spin();

    return 0;
}

