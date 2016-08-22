#include <ros/ros.h>
#include <ros/node_handle.h>

#include <pcl/point_types.h>
#include <pcl/conversions.h>

#include <octomap_server/OctomapServer.h>

#include <message_filters/subscriber.h>

#include <sensor_msgs/PointCloud2.h>

typedef pcl::PointXYZDGI PCLPoint1;
typedef pcl::PointXYZRGB PCLPoint2;
typedef pcl::PointCloud<pcl::PointXYZDGI> MyPointCloud1;
typedef pcl::PointCloud<pcl::PointXYZRGB> MyPointCloud2;

class RGBServer {
public:
    //Constructor
    RGBServer() {
        ROS_INFO("RGB: IN SERVER\n");
        onInit();
    }

private:

    virtual void onInit() {

        ROS_INFO("RGB: IN INIT\n");

        ros::NodeHandle nh; 

        std::string topic = "/dgi_points";

        // queue size
        int queue_size = 2;

        //Topic you want to publish
        pub_ = nh.advertise<sensor_msgs::PointCloud2>("rgb_points", 1000);

        ROS_INFO("RGB: Calling callback\n");
        sub_= nh.subscribe<sensor_msgs::PointCloud2> (topic, queue_size, &RGBServer::callback, this);

    }

    PCLPoint2 get_RGB_point(PCLPoint1 in) {
        PCLPoint2 pt;
        pt.x = in.x;
        pt.y = in.y;
        pt.z = in.z;
        pt.r = in.g;
        pt.g = (uint8_t)100;
        pt.b = (uint8_t)100;
        return pt;
    }

    // callback signature
    void callback(const sensor_msgs::PointCloud2ConstPtr& pc_in) {
        
        MyPointCloud1 pc_in_temp;

        // void fromROSMsg(const sensor_msgs::PointCloud2 &cloud, pcl::PointCloud<T> &pcl_cloud)

        //pcl::PCLPointCloud2 pcl_pc2;
        //pcl_conversions::toPCL(cloud, pcl_pc2);
        //pcl::fromPCLPointCloud2(pcl_pc2, pcl_cloud);
        fromROSMsg(*pc_in, pc_in_temp);

        MyPointCloud2 pc_out_temp;
        sensor_msgs::PointCloud2 pc_out;

        pc_out_temp.header = pc_in_temp.header;

        pc_out_temp.height = pc_in_temp.height;
        pc_out_temp.width = pc_in_temp.width;

        float maxX, maxY, maxZ, minX, minY, minZ;
        /*
          maxX = -1000;
        maxY = -1000;
        maxZ = -1000;
        minX = 1000;
        minY = 1000;
        minZ = 1000;
        */
        int i, j;
        for (i=0; i < pc_in_temp.height; i++) {
            for (j=0; j < pc_in_temp.width; j++) {

                PCLPoint2 pt = get_RGB_point(pc_in_temp.points[i * pc_in_temp.width + j]);
                /*
                if (pt.x > maxX) {
                    maxX = pt.x;
                }

                if (pt.y > maxY) {
                    maxY = pt.y;
                }

                if (pt.z > maxZ) {
                    maxZ = pt.z;
                }

                if (pt.x < minX) {
                    minX = pt.x;
                }

                if (pt.y < minY) {
                    minY = pt.y;
                }

                if (pt.z < minZ) {
                    minZ = pt.z;
                }
                */
                pc_out_temp.points.push_back(pt);
            }
        }

        toROSMsg(pc_out_temp, pc_out);
        pub_.publish(pc_out);

        //        ROS_INFO("\nmaxX: %f\nmaxY: %f\nmaxZ: %f\nminX: %f\nminY: %f\nminZ: %f\n", maxX, maxY, maxZ, minX, minY, minZ);
    }

    ros::Subscriber sub_;
    ros::Publisher pub_;

};//End of class RGBServer

int main(int argc, char **argv) {

    //Initiate ROS
    ros::init(argc, argv, "rgb_server_node");

    //Create an object of class RGBServer that will take care of everything
    RGBServer rgb_server;

    ros::spin();

    return 0;
}

