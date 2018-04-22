//intent of this fnc: read a PCD file, convert it to a 2-D gray-scale image,
// where brightness is (piecewise) linearly related to proximity
// see if the resulting images adequately encode depth as grayscale
// (could also display as false-color depth)


#include <ros/ros.h>
#include <image_transport/image_transport.h>
//#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h> //generic C++ stuff
#include <stdlib.h>
#include <math.h>

#include <sensor_msgs/PointCloud2.h> 
#include <pcl_ros/point_cloud.h> //to convert between PCL and ROS
#include <pcl/conversions.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
//#include <pcl/PCLPointCloud2.h> //PCL is migrating to PointCloud2 

#include <pcl/common/common_headers.h>
#include <pcl-1.7/pcl/point_cloud.h>
#include <pcl-1.7/pcl/PCLHeader.h>

using namespace std;
//will use filter objects "passthrough" and "voxel_grid" in this example
//#include <pcl/filters/passthrough.h>
//#include <pcl/filters/voxel_grid.h> 

//black(0) means "not relevant", e.g. table (sensed, but not part of object)
//also include a back-drop?  so can also get data from this, but knowably not part of object
//"holes" in the scene are unknown; fill in with mean val? min val? 
//unsigned bytes for intensity



//some parameters to be tuned for image projection computation
//choose the resolution for projected image
int Nv = 50;
int Nu = 50;
double focal_len = 520; //220 may be good; // focal length, in pixels

double z_min = 0.0; // min dist = closest approach of imaged objects; will assign intensity=255 at this dist
double z_max = 0.1; // max dist = furthest point of interest on object; will assign intensity=100 at this dist

//cv::Mat g_image(Nu,Nv,CV_8U,cv::Scalar(50)); //create image, set encoding and size, init pixels to default val
cv::Mat g_image(Nu, Nv, CV_8U, cv::Scalar(0)); //create image, set encoding and size, init pixels to default val

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_converter");
    //load a PCD file using pcl::io function; alternatively, could subscribe to Kinect messages    
    string fname;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclKinect_clr_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //pointer for color version of pointcloud

    cout << "enter pcd file name: "; //prompt to enter file name
    cin >> fname;
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (fname, *pclKinect_clr_ptr) == -1) //* load the file
    {
        ROS_ERROR("Couldn't read file \n");
        return (-1);
    }
    int npts_cloud = pclKinect_clr_ptr->width * pclKinect_clr_ptr->height;
    int width = pclKinect_clr_ptr->width;
    int height = pclKinect_clr_ptr->height;
    std::cout << "Loaded "
            << npts_cloud
            << " data points from file " << fname << std::endl;
    Eigen::Vector3f cloud_pt;
    double x, y, z;
    double v, u;

    uchar gray_level;
    double r;
    //projection:  given point (x,y,z) in CAMERA space (possibly result of filtering and zooming)
    //compute gray-scale value based on z-coord;
    //assign this value to pixel (i,j) in 2-D image based on: x/z = (u-u_c)/fx (where u and f are in pixels)
    //  y/z = (v-v_c)/fy
    // choose Nu=u_max, Nv=v_max based on desired resolution,
    //and choose focal pt s.t.:  (x_max-x_min)/z_min = u_max/fx;     (y_max-y_min)/z_min = v_max/fy
    // choose x_max, x_min, y_max, y_min, z_max, z_min for (box) region of interest
    //(nominal viewing platform)  
    //while (z_min>0) {
    cout << "enter z_min: ";
    cin>>z_min;
    cout << "enter z_max: ";
    cin>>z_max;
    for (int ipt = 0; ipt < npts_cloud; ipt++) {
        cloud_pt = pclKinect_clr_ptr->points[ipt].getVector3fMap();
        z = cloud_pt[2];
        y = cloud_pt[1];
        x = cloud_pt[0];
        if ((z == z)&&(x == x)&&(y == y)) { // test for Nan 
            u = ipt % width; //uc + focal_len*x/z;
            //i = round(u);
            v = (int) (ipt / width); //vc + focal_len*y/z;
            //j = round(v);
            gray_level = (uchar) (255 * (z_max - z) / (z_max - z_min));

            // convert z to an intensity:
            if (z > z_max) gray_level = 255;
            else if (z < z_min) gray_level = 0;
            else {
                gray_level = (uchar) (255 * (z_max - z) / (z_max - z_min));
            }
            g_image.at<uchar>(u, v) = gray_level;
            //cv_ptr->image.at<cv::Vec3b>(j, i)[0] = 0;

        }

    }
    std::cout << "output image size: " << g_image.size().height << " , "
            << g_image.size().width << std::endl; //initially, 0x0
    //cv::namedWindow("Original Image"); // define the window
    //cv::imshow("Original Image", g_image); // show the image

    cv::imwrite("output.bmp", g_image);
    //ros::spin();
    return 0;
}
