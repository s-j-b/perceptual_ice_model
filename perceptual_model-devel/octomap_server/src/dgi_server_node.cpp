#include "ros/ros.h"
#include <ros/node_handle.h>

#include "pluginlib/class_list_macros.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <depth_image_proc/depth_traits.h>

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/CameraInfo.h>

#include <octomap_server/OctomapServer.h>

#include <time.h>

#define X_OFF (212)
#define Y_OFF (235)
#define W (761)
#define H (470)

typedef pcl::PointXYZDGI PCLPoint;
typedef pcl::PointCloud<pcl::PointXYZDGI> MyPointCloud;

/**
 * DGIServer: class and node definition
 *
 * The DGI Server subscribes to direct, global, intensity, and depth images,
 * along with a camera-info topic. From these, a DGI Point Cloud
 * (see include/extra_point_types.h) is created and published
 *
 */
class DGIServer {
public:
    //Constructor
    DGIServer() {
        onInit();
    }

private:

    virtual void onInit() {

        // Declare and initialize node handlers
        // (Multiple node handlers for modularity,
        // simplifies debugging synchronization)
        ros::NodeHandle nh;
        ros::NodeHandle private_nh;

        ros::NodeHandle direct_nh(nh, "direct");
        ros::NodeHandle global_nh(nh, "global");
        ros::NodeHandle intensity_nh(nh, "intensity");
        ros::NodeHandle depth_nh(nh, "depth");

        ros::NodeHandle direct_pnh(private_nh, "direct");
        ros::NodeHandle global_pnh(private_nh, "global");
        ros::NodeHandle intensity_pnh(private_nh, "intensity");
        ros::NodeHandle depth_pnh(private_nh, "depth");

        // Image Transport init
        image_transport::ImageTransport depth_it(depth_nh);
        image_transport::ImageTransport direct_it(direct_nh);
        image_transport::ImageTransport global_it(global_nh);
        image_transport::ImageTransport intensity_it(intensity_nh);

        // Transport Hints init
        image_transport::TransportHints hintsDepth("raw", ros::TransportHints(), depth_pnh);
        image_transport::TransportHints hintsDirect("raw", ros::TransportHints(), direct_pnh);
        image_transport::TransportHints hintsGlobal("raw", ros::TransportHints(), global_pnh);
        image_transport::TransportHints hintsIntensity("raw", ros::TransportHints(), intensity_pnh);

        int queueSize = 5;

        // Create our exactSync filter, which listens to the 5 specified subscribed topics
        ROS_INFO("DGI: SETUP SYNC\n");
        exactSync_ = new message_filters::Synchronizer<MyExactSyncPolicy>(MyExactSyncPolicy(queueSize),
                                                                            info_sub_,
                                                                            depth_sub_,
                                                                            direct_sub_,
                                                                            global_sub_,
                                                                            intensity_sub_);        

        std::string base_topic_name = "/episcan";
        std::string base_topic_name_republish = "/episcan_republish";
        std::string topic_type = "/image_mono";

        info_sub_.subscribe(direct_nh,
                            base_topic_name + "/depth/camera_info",
                            queueSize);        

        depth_sub_.subscribe(depth_it,
                             base_topic_name + "/depth/image_rect",
                             queueSize,
                             hintsDepth);
        direct_sub_.subscribe(direct_it,
                              base_topic_name + "/direct" + topic_type,
                              queueSize,
                              hintsDirect);
        global_sub_.subscribe(global_it,
                              base_topic_name + "/indirect" + topic_type,
                              queueSize,
                              hintsGlobal);
        intensity_sub_.subscribe(intensity_it,
                                 base_topic_name + "/global" + topic_type,
                                 queueSize,
                                 hintsIntensity);

        // Initialize DGI Point Cloud publisher
        pub_ = nh.advertise<sensor_msgs::PointCloud2>("dgi_points", 1000);

        // Call the DGI processing callback, from which we publish
        exactSync_->registerCallback(boost::bind(&DGIServer::callback, this,
                                                  _1, _2, _3, _4, _5));

    };

    PCLPoint get_DGI_point(int u, int v,
                           float depth,
                           uint8_t direct, uint8_t global, uint8_t intensity,
                           float fx, float fy, float cx, float cy) {
    
        float x, y, z;
        x = (((float)u - cx) * depth) / fx;
        y = (((float)v - cy) * depth) / fy;

        // Set Pointcloud 2 fields
        return PCLPoint(x, y, depth, direct, global, intensity);
    }

    void callback(const sensor_msgs::CameraInfoConstPtr& camera_info,
                  const sensor_msgs::ImageConstPtr& depth_image, 
                  const sensor_msgs::ImageConstPtr& direct_image,
                  const sensor_msgs::ImageConstPtr& global_image,
                  const sensor_msgs::ImageConstPtr& intensity_image) {
        
        // V-- Uncomment to check arrival in callback
        std::string episcan = "episcan";

        memcpy(direct_image->header.frame_id, episcan.c_str, 7 * sizeof(char));
        memcpy(global_image->header.frame_id, episcan, 7 * sizeof(char));
        memcpy(intensity_image->header.frame_id, episcan, 7 * sizeof(char));

        time_t t1, t2;

        time(&t1);

        MyPointCloud pc;
        sensor_msgs::PointCloud2 pc_2;

        pc.header = pcl_conversions::toPCL(depth_image->header);
        pc.height = H;
        pc.width = W;
        pc.is_dense = false;

        image_geometry::PinholeCameraModel model_;
        model_.fromCameraInfo(camera_info);

        // Use correct principal point from calibration
        float center_x = (float)model_.cx();
        float center_y = (float)model_.cy();

        // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
        //double unit_scaling = depth_image_proc::DepthTraits<float>::toMeters( float(1.0) );

        float unit_scaling = 1.0f;
        float constant_x = unit_scaling / (float)model_.fx();
        float constant_y = unit_scaling / (float)model_.fy();
        float bad_point = std::numeric_limits<float>::quiet_NaN ();
  
        const float* depth_row = reinterpret_cast<const float*>(&depth_image->data[0]);
        const uint8_t* d = reinterpret_cast<const uint8_t*>(&direct_image->data[0]);
        const uint8_t* g = reinterpret_cast<const uint8_t*>(&global_image->data[0]);
        const uint8_t* i = reinterpret_cast<const uint8_t*>(&intensity_image->data[0]);

        int row_step = depth_image->step / sizeof(float);
        int dgi_skip = direct_image->step / sizeof(uint8_t);

        float depth;
        float x, y, z;

        for (int v = 0; v < int(depth_image->height); ++v, depth_row += row_step,
                 d += dgi_skip, g += dgi_skip, i += dgi_skip) {
            for (int u = 0; u < int(depth_image->width); ++u) {
                
                if ((v >= Y_OFF) && (v < Y_OFF + H) && (u >= X_OFF) && (u < X_OFF + W)) {
                    depth = depth_row[u];
                
                    // Check for invalid measurements



                    if ((!std::isfinite(depth)) || (depth < 0.5 || depth > 1.0)) {
                        x = y = z = bad_point;
                    } else {

                        // Fill in XYZ
                        x = (u - center_x) * depth * constant_x;
                        y = (v - center_y) * depth * constant_y;
                        z = depth;
                    }

                    pc.points.push_back(PCLPoint(x, y, z, d[u], g[u], i[u]));
                }
            }
        }
        time(&t2);

          struct tm y2k = {0};
          y2k.tm_hour = 0;   y2k.tm_min = 0; y2k.tm_sec = 0;
          y2k.tm_year = 100; y2k.tm_mon = 0; y2k.tm_mday = 1;

        double secs = difftime(t2, t1);
        double abs = difftime(t1, mktime(&y2k));
        //        ROS_INFO("DIFF SECS: %.f\n", secs);
        //        ROS_INFO("ABS SECS: %.f\n", abs);

        toROSMsg(pc, pc_2);
        pub_.publish(pc_2);
    }

    // image_transport::Publisher imageDepthPub_;
    ros::Publisher pub_;

    image_transport::SubscriberFilter direct_sub_;
    image_transport::SubscriberFilter global_sub_;
    image_transport::SubscriberFilter intensity_sub_;
    image_transport::SubscriberFilter depth_sub_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub_;

    typedef message_filters::sync_policies::ExactTime<sensor_msgs::CameraInfo,
                                                      sensor_msgs::Image,
                                                      sensor_msgs::Image,
                                                      sensor_msgs::Image,
                                                      sensor_msgs::Image> MyExactSyncPolicy;

    message_filters::Synchronizer<MyExactSyncPolicy> * exactSync_;
        
};

int main(int argc, char **argv) {

    //Initiate ROS
    ros::init(argc, argv, "dgi_server_node");

    ROS_INFO("DGI: IN MAIN\n");

    //Create an object of class SubscribeAndPublish that will take care of everything
    DGIServer dgi_server;

    ros::spin();

    return 0;
}
