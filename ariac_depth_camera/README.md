# ariac_depth_camera

package to initiate processing of ARIAC depth-camera images.


## Example usage
can get live images from ARIAC with: 
roslaunch osrf_gear sample_environment.launch fill_demo_shipment:=true

or:
rosrun osrf_gear gear.py -f `catkin_find --share --first-only osrf_gear`/config/quals/qual2b.yaml ~/ariac_ws/src/cwru_ariac_2018/copy_of_team_config/depthcam_config.yaml

This starts up an ARIAC simulation that includes depth cameras viewing bins of parts. 
The topics for thes cameras are: "/ariac/depth_camera_1" through "/ariac/depth_camera_1".

With images being published, these can be displayed in Rviz by adding a PointCloud item, and selecting the topic to be, e.g., 
"/ariac/depth_camera_1".  

In the present package, there are two nodes: "pcd_snapshot_ariac" and "interpret_pcd_file".  The first node is similar to
its counterpart in "learning_ros", with some important internal differences.  This node subscribes to the "/ariac/depth_camera_1"
topic, receives a message of type sensor_msgs::PointCloud, then converts this to a PointCloud2 before saving it to disk
under the filename "depthcam_snapshot.pcd".  As part of this conversion, the PointCloud2 is informed that this is an
"organized" point cloud by specifying height=50 and width=50.  (To access the other depth cameras, change the topic name).  

The saved file can be read from disk, analyzed and displayed in Rviz using "interpret_pcd_file":
`rosrun ariac_depth_camera interpret_pcd_file`
The user is prompted to enter a PCD pointcloud image file name.  The analysis performs the following.  It transforms the points
to a "bin" frame in which the z-axis is perpendicular to the bin surface and the frame origin is on the bin surface.  Points with
z values greater than 0 correspond to points belonging to parts in the bin.  The z values of part samples are averaged, yielded
a part thickness value.  This information is enough to distinguish among the 5 part types.  The transformed points are 
used to synthesize a 2-D grayscale image (a binary image is a viable alternative).  Points on the bin surface are encoded as black, 
and points on the part's top surface are encoded as white.  The resulting 2-D image is saved as "output.bmp".  Finally, 
this node goes into a loop, publishing the point-cloud image read from disk (both in the camera frame and in the bin frame).


