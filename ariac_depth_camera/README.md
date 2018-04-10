# ariac_depth_camera

package to initiate processing of ARIAC depth-camera images.


## Example usage
can get live images from ARIAC with: 
roslaunch osrf_gear sample_environment.launch fill_demo_shipment:=true

(Alternatively, playback the bagfile: depth_cam_pubs.bag)

This starts up an ARIAC simulation that includes a depth camera viewing a bin with gasket parts.
The topic for this camera is: "/ariac/depth_camera_1", which has an older, PointCloud message type.

With images being published, can display these in Rviz by adding a PointCloud item, and selecting the topic to be
"/ariac/depth_camera_1".  

If using rosbag playback, it will be necessary to choose the pointcloud's "frame_id" as the fixed frame.  This frame_id
is: "depth_camera_1_frame"

In the present package, there are two nodes: "pcd_snapshot_ariac" and "display_pcd_file".  These first node is similar to
its counterpart in "learning_ros", with some important internal differences.  This node subscribes to the "/ariac/depth_camera_1"
topic, receives a message of type sensor_msgs::PointCloud, then converts this to a PointCloud2 before saving it to disk
under the filename "depthcam_snapshot.pcd".  A part of this conversion, the PointCloud2 is informed that this is an
"organized" point cloud by specifying height=50 and width=50.

The saved file can be read from disk and displayed in Rviz using "display_pcd_file".  This is merely a copy of the
same node from "learning_ros".  Run as:
`rosrun ariac_depth_camera display_pcd_file`
and at the prompt, enter the file name (e.g.  "depthcam_snapshot.pcd")
Note that to view the corresponding images, Rviz will have to have its fixed_frame set
to "depth_camera_1_frame".




    
