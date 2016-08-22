#include <ros/ros.h>

#include <octomap_server/OctomapServer.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <image_transport/image_transport.h>
#include <image_transport/camera_subscriber.h>


typedef pcl::PointXYZDGI PCLPoint;
typedef pcl::PointCloud<pcl::PointXYZDGI> PointCloud;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, 
                                                        sensor_msgs::Image,
                                                        sensor_msgs::Image,
                                                        sensor_msgs::Image,
                                                        sensor_msgs::CameraInfo> SyncPolicy;

class SubscribeAndPublish {
public:

    SubscribeAndPublish() {

        std::string base_topic_name = "/episcan";
        std::string topic_type = "/image_mono";
        // queue size
        int q = 1;

        //Topic you want to publish
        pub_ = nh_.advertise<sensor_msgs::PointCloud2>("dgi_points", 1000);

        //Topics to which you want to subscribe
        direct_sub_ = const message_filters::Subscriber<sensor_msgs::Image>(nh_,
                                                                            base_topic_name + "/direct" + topic_type,
                                                                            q);

        global_sub_ = const message_filters::Subscriber<sensor_msgs::Image>(nh_,
                                                                            base_topic_name + "/global" + topic_type,
                                                                            q);

        intensity_sub_ = const message_filters::Subscriber<sensor_msgs::Image>(nh_,
                                                                               base_topic_name + "/intenisty" + topic_type,
                                                                               q);

        depth_sub_ = const message_filters::Subscriber<sensor_msgs::Image>(nh_,
                                                                           base_topic_name + "/depth" + topic_type,
                                                                           q);

        cinfo_sub_ = const message_filters::Subscriber<sensor_msgs::CameraInfo>(nh_,
                                                                                base_topic_name + "/left/camera_info", 
                                                                                q);

        
        sync_ = message_filters::Synchronizer<SyncPolicy>(SyncPolicy(q),
                                                          direct_sub,
                                                          global_sub,
                                                          intensity_sub,
                                                          depth_sub,
                                                          cinfo_sub);

        sync_.registerCallback(boost::bind(&callback, _1, _2, _3, _4, _5, _6));

    }

    PCLPoint get_DGI_point(tf::Matrix3x3 m, int i, int j, float depth,
                           uint8_t direct, uint8_t global, uint8_t intensity) {
    
        float x, y, z;
        x = ((float)i * m[0][0] + m[0][2]) * depth;
        y = ((float)j * m[1][1] + m[1][2]) * depth;
        return PCLPoint(x, y, depth, direct, global, intensity);
    }



    void callback(const sensor_msgs::ImageConstPtr& direct_image,
                  const sensor_msgs::ImageConstPtr& global_image,
                  const sensor_msgs::ImageConstPtr& intensity_image,
                  const sensor_msgs::ImageConstPtr& depth_image, 
                  const sensor_msgs::CameraInfoConstPtr& camera_info) {


        tf::Matrix3x3 P(camera_info->P[0],
                        camera_info->P[1],
                        camera_info->P[2],
                        camera_info->P[4],
                        camera_info->P[5],
                        camera_info->P[6],
                        camera_info->P[8],
                        camera_info->P[9],
                        camera_info->P[10]);
        tf::Matrix3x3 P_i = P.inverse();
    
        // Perception
    
        PointCloud pc;
        sensor_msgs::PointCloud2 pc2;

    
        pc.header.frame_id = depth_image->header.frame_id;
        pc.height = depth_image->height;
        pc.width = depth_image->width;

        int i, j;
        for (i=0; i < pc.height; i++) {
            for (j=0; j < pc.width; j++) {
                pc.points.push_back(get_DGI_point(P_i, i, j,
                                                  depth_image->data[i * depth_image->width + j],
                                                  direct_image->data[i * depth_image->width + j],
                                                  global_image->data[i * depth_image->width + j],
                                                  intensity_image->data[i * depth_image->width + j]));
            }
        }

        toROSMsg(pc, pc2);
        pub_.publish(pc2);
    }


private:
    ros::NodeHandle nh_; 
    ros::Publisher pub_;

    const message_filters::Subscriber<sensor_msgs::Image> direct_sub_;
    const message_filters::Subscriber<sensor_msgs::Image> global_sub_;
    const message_filters::Subscriber<sensor_msgs::Image> intensity_sub_;
    const message_filters::Subscriber<sensor_msgs::Image> depth_sub_;
    const message_filters::Subscriber<sensor_msgs::CameraInfo> cinfo_sub_;

    message_filters::Synchronizer<SyncPolicy> sync_;

};//End of class SubscribeAndPublish

int main(int argc, char **argv) {
    //Initiate ROS
    ros::init(argc, argv, "dgi_server_node");

    //Create an object of class SubscribeAndPublish that will take care of everything
    SubscribeAndPublish SAPObject;

    ros::spin();

    return 0;
}
