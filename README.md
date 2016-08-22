# perceptual_ice_model
Episcan Perceptual Model (Ice perception + mapping)
Simon J. Bloch
simon.j.bloch@gmail.com
- - - - - - - - - - - - - - - - - - - - - - - - - -

Instructions:

Install ROS, install octomap kinetic

Copy the following into your catkin_ws/src directory:
     - perceptual_model-devel
     - octomap_msgs-kinetic-devel
     - octomap_rviz_plugins-indigo-devel

Copy the octomap-devel into your home directory or another dir
     - Update the octomap PATH variable to identify the location of octomap-devel

Compile octomap-devel (if not already compiled)

Compile your catkin workspace

RUN CODE:

All bag files are in run/bags
    -Change the directory in stereo_view.rviz to accommodate your path
    (e.g. home/jupiter/run/...)

    RUN IN DIFFERENT WINDOWS
    -roslaunch playback.launch 
               (opens rviz window)
    -rosbag play --clock -l bags/some_bagfile_name.bag
               (plays bagfile)
    -roslaunch dgi.launch
               (processes bagfile and outputs dgi and dgip point clouds)
    -roslaunch episcan.launch
               (processes dgip point cloud into an episcan octomap
               

CODE ORGANIZATION:
     -Most code is in the perceptual_model-devel ROS package, in:
           octomap_server/src:
                - OctomapServer.cpp (main server code!)
                - dgi_server_node.cpp
                - dgip_server_node.cpp
                - rgb_server_node.cpp (for testing)

           octomap_server/include/octomap_server
                - OctomapServer.h (contains sensor model function! called in .cpp file)
                - extra_point_types.h (definition of dgi point)
           
     -supplemental RVIZ code and other stuff is in the other packages
           octomap_rviz_plugins-indigo-devel/src
                - occupancy_grid_display.cpp

     -EpiscanOcTree code is in octomap-devel

CURRENT KNOWN BUGS:
     -There is an issue with transforms not being adequately published because some of the
     recorded topics in the bagfiles don't have specified frame_ids (due to a problem in the
     sensor-side code)