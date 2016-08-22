#include "ros/ros.h"
#include "pluginlib/class_list_macros.h"
#include "nodelet/nodelet.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <sensor_msgs/CameraInfo.h>

#include <cv_bridge/cv_bridge.h>

#include <rtabmap/core/util2d.h>

namespace rtabmap_ros
{

    class SubscribeAndPublish {
    public:
        //Constructor
        SubscribeAndPublish() {}

    private:

        virtual void onInit()
        {
            ros::NodeHandle& nh = getNodeHandle();
            ros::NodeHandle& private_nh = getPrivateNodeHandle();

            ros::NodeHandle direct_nh(nh, "direct");
            ros::NodeHandle global_nh(nh, "global");
            ros::NodeHandle intensity_nh(nh, "intensity");
            ros::NodeHandle depth_nh(nh, "depth");

            ros::NodeHandle direct_pnh(private_nh, "direct");
            ros::NodeHandle global_pnh(private_nh, "global");
            ros::NodeHandle intensity_pnh(private_nh, "intensity");
            ros::NodeHandle depth_pnh(private_nh, "depth");

            image_transport::ImageTransport direct_it(direct_nh);
            image_transport::ImageTransport global_it(global_nh);
            image_transport::ImageTransport intensity_it(intensity_nh);
            image_transport::ImageTransport depth_it(depth_nh);

            image_transport::TransportHints hintsDirect("raw", ros::TransportHints(), direct_pnh);
            image_transport::TransportHints hintsGlobal("raw", ros::TransportHints(), global_pnh);
            image_transport::TransportHints hintsIntensity("raw", ros::TransportHints(), intensity_pnh);
            image_transport::TransportHints hintsDepth("raw", ros::TransportHints(), depth_pnh);

            int queueSize = 10;
            bool approxSync = true;

            approxSync_ = new message_filters::Synchronizer<MyApproxSyncPolicy>(MyApproxSyncPolicy(queueSize),
                                                                                direct_sub_,
                                                                                global_sub_,
                                                                                intensity_sub_,
                                                                                depth_sub_,
                                                                                info_sub_);

            approxSync_->registerCallback(boost::bind(&PublisherAndSubscriber::callback, this,
                                                      _1, _2, _3, _4, _5));


            image_sub_.subscribe(rgb_it, rgb_nh.resolveName("image_in"), 1, hintsRgb);

            direct_sub_.subscribe(direct_it, direct_nh.resolveName("image_in"), 1, hintsDirect);
            global_sub_.subscribe(global_it, global_nh.resolveName("image_in"), 1, hintsGlobal);
            intensity_sub_.subscribe(intensity_it, intensity_nh.resolveName("image_in"), 1, hintsIntensity);
            depth_sub_.subscribe(depth_it, depth_nh.resolveName("image_in"), 1, hintsDepth);
            info_sub_.subscribe(direct_nh, "camera_info_in", 1);

            ////////////////////////////////////////////////////////////////////////////////

            ////////////////////////////////////////////////////////////////////////////////

            ////////////////////////////////////////////////////////////////////////////////

            pub_ = depth_it.advertise<sensor_msgs::PointCloud2>("dgi_points", 1000);
        };

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

            /*
            imagePub_.publish(image);
            
             
            sensor_msgs::CameraInfo info = *camInfo;
            infoPub_.publish(info);
            */
        }

        // image_transport::Publisher imageDepthPub_;
        ros::Publisher pub_;

        image_transport::SubscriberFilter direct_sub_;
        image_transport::SubscriberFilter global_sub_;
        image_transport::SubscriberFilter intensity_sub_;
        image_transport::SubscriberFilter depth_sub_;
        message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub_;

        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                                sensor_msgs::Image,
                                                                sensor_msgs::Image,
                                                                sensor_msgs::Image,
                                                                sensor_msgs::CameraInfo> MyApproxSyncPolicy;

        message_filters::Synchronizer<MyApproxSyncPolicy> * approxSync_;
        
    };
}
