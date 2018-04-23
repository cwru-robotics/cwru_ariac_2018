//analyze_pcd_file.cpp
// prompts for a pcd file name, reads the file, and displays to rviz on topic "pcd"
// tries to find "interesting" points above the support surface
//hard-code transform between bin and camera
//wsn April 2018

#include<ros/ros.h> //generic C++ stuff
#include <stdlib.h>
#include <math.h>
#include <sensor_msgs/PointCloud2.h> //useful ROS message types
#include <pcl_ros/point_cloud.h> //to convert between PCL a nd ROS

//#include <pcl/ros/conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common_headers.h>
#include <pcl-1.7/pcl/point_cloud.h>
#include <pcl-1.7/pcl/PCLHeader.h>
#include <pcl_utils/pcl_utils.h>

//for opencv
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>


using namespace std;

int main(int argc, char** argv) {
    ros::init(argc, argv, "pcd_interpreter"); //node name
    ros::NodeHandle nh;

    //hard-coded identification of bin plane from depth camera:
    //these were found from a plane fit of a camera viewing an empty bin;
    // expect same transform for all  5 bins
    Eigen::Vector3f plane_normal;
    double nom_tilt_ang = atan2(-0.285, -0.958);

    plane_normal << cos(nom_tilt_ang), sin(nom_tilt_ang), 0; //-0.958, -0.285, 0;
    double plane_dist = -0.4675;

    //library that provides some helper functions:
    PclUtils pclUtils(&nh);
    //create a coordinate frame on the identified plane using the plane's parameters
    Eigen::Affine3f affine_plane_frame;
    affine_plane_frame = pclUtils.make_affine_from_plane_params(plane_normal, plane_dist);


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_clr_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //pointer for color version of pointcloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //output cloud
    cout << "enter pcd file name: ";
    string fname;
    cin>>fname;

    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (fname, *pcl_clr_ptr) == -1) //* load the file
    {
        ROS_ERROR("Couldn't read file \n");
        return (-1);
    }
    std::cout << "Loaded "
            << pcl_clr_ptr->width * pcl_clr_ptr->height
            << " data points from file " << fname << std::endl;

    //    pcl::transformPointCloud(*pclKinect_clr_ptr, *transformed_cloud_ptr, A_plane_wrt_camera.inverse());
    pcl::transformPointCloud(*pcl_clr_ptr, *output_cloud_ptr, affine_plane_frame.inverse());

    //pclUtils.transform_cloud(affine_plane_frame, pcl_clr_ptr,output_cloud_ptr);
    int npts_cloud = output_cloud_ptr->width * output_cloud_ptr->height;
    int width = output_cloud_ptr->width;
    int height = output_cloud_ptr->height;
    ROS_INFO("output cloud has %d points, %d x %d", npts_cloud, width, height);

    //analyze the transformed cloud:
    //evaluate how many 3-D points lie within 1mm slabs parallel to the bin surface
    double dz = 0.001;
    double z, zmax;
    int npts;
    double z_sum = 0.0;
    Eigen::Vector3f cloud_pt;
    int ntop = 0;
    //sort the z-values of points in the range from -2cm to +3cm w/rt bin surface
    for (double zmin = -0.0205; zmin < 0.03; zmin += dz) {
        npts = 0;
        zmax = zmin + dz;
        for (int ipt = 0; ipt < npts_cloud; ipt++) {
            //eval z-coord of every point
            cloud_pt = output_cloud_ptr->points[ipt].getVector3fMap();
            z = cloud_pt[2];
            //does it lie between the current upper and lower bounds?
            // if so, increment the count of points in this range
            if ((z > zmin)&&(z < zmax)) npts++;
        }
        ROS_INFO("at zmin = %f, zmax = %f: npts = %d", zmin, zmax, npts);

    }

    //search through the points again: how many points have z-values that are
    // above the bin surface?  What is the average of these z values?
    for (int ipt = 0; ipt < npts_cloud; ipt++) {
        cloud_pt = output_cloud_ptr->points[ipt].getVector3fMap();
        z = cloud_pt[2];
        if (z > 0.0) {
            ntop++;
            z_sum += z;
        }
    }
    double z_top = z_sum / ntop; //compute average z value for z values>0
    ROS_INFO("mean z for values>0 = %f", z_top);

    //convert the 3-D image to a 2-D image, where illumination is proportional to z-value=
    // height above plane;
    // Alternatively, create a binary 2-d image where z values >0 map to 1, all others to 0
    
    //choose the resolution for projected image;
    //source is 50x50, BUT resolution is higher in y-direction than x-direction;
    //need to scale transform to pixels correspondingly so circular objects are
    //circular in the projection
    int Nv = 50; //number of pixels of synthesized image in vertical direction
    int Nu = 35; //number of pixels in horizontal direction
    double min_y = -0.229132; //these values were observed from camera outputs
    double max_y = 0.233651;
    double min_x = -0.005231;
    double max_x = 0.315519;
    double zval_black = 0.0; // in ouput image, assign intensity=0 at this z-value (or less)
    double zval_white = 0.01; //in output image, assign intensity=255 at this z-value (or above)      
    int u, v;
    //point-cloud data is in units of meters, but image will be indexed by pixels;
    // scale factor converts meters to pixels; will need to go back in other direction as well
    double meters_to_pixels = 110; //100 //convert pointcloud data to pixel numbers
    cv::Mat depthmap_image(Nu, Nv, CV_8U, cv::Scalar(0)); //create image, set encoding and size, init pixels to default val
    uchar gray_level;
    double x, y;
    //for each pointcloud point, compute which pixel it maps to and find its z value
    for (int ipt = 0; ipt < npts_cloud; ipt++) {
        cloud_pt = output_cloud_ptr->points[ipt].getVector3fMap();
        z = cloud_pt[2];
        y = cloud_pt[1];
        x = cloud_pt[0];
        //careful: some pointclouds include bad data; test for this
        if ((z == z)&&(x == x)&&(y == y)) { // test for Nan 
            //compute pixel values from metric values
            v = round((y - min_y) * meters_to_pixels);
            u = round((x - min_x) * meters_to_pixels);
            //flip/invert these so image makes sense visually
            u = Nu - u;
            v = Nv - v;
            // convert z to an intensity:
            if (z > zval_white) gray_level = 255; //max intensity
            else if (z < zval_black) gray_level = 0; //min intensity
            else {
                gray_level = (uchar) (255 * (z - zval_black) / (zval_white - zval_black));
            }
            if ((u > 0)&&(u < Nu - 1)&&(v > 0)&&(v < Nv - 1)) {
                depthmap_image.at<uchar>(u, v) = gray_level;
            }
        }

    }
    //ROS_INFO("min_x,max_x, min_y,max_y = %f, %f, %f, %f",min_x,max_x,min_y,max_y);
    std::cout << "output image size: " << depthmap_image.size().height << " , "
            << depthmap_image.size().width << std::endl; //initially, 0x0
    //save the synthesized  image to  disk
    cv::imwrite("output.bmp", depthmap_image);
    
    //interpret image w/rt template:
    //see OpenCV: void matchTemplate(InputArray image, InputArray templ, OutputArray result, int method)
	//cv::Mat src_img, template_img;
	//cv::Mat result_mat;
    int Ntu = 23; //define template 10x10 (arbitrary)
    int Ntv = 23; 
    int tu_c = 11; //(7,7) is central pixel of template 15x15 template
    int tv_c = 11;
    int template_u_halfwidth = 11; //template is +/-5 pixels about center
    int template_v_halfwidth = 11;
    
    //here is the pattern of interest: a circle of diameter Ntu = Ntv pixels
    //this template is for the "disk" part
    cv::Mat template_img(Ntu, Ntv, CV_8U, cv::Scalar(0)); //create image, set encoding and size, init pixels to default val
    int Rpix = 7; //make a template of a circle of radius 7 pixels
    double r,theta;
    for (r=0;r<=Rpix;r+=0.2) {
        for (theta=0;theta<6.28;theta+=0.01) {
            u= round(r*sin(theta))+tu_c;
            v= round(r*cos(theta))+tv_c;
            cout<<"tu= "<<u<<", tv = "<<v<<endl;
            template_img.at<uchar>(u, v) = 255;
        }
    }
    /*
    for (u=2;u<2+hu;u++) {
        for (v=2;v<2+wv;v++) {
            template_img.at<uchar>(u, v) = 255;
        }
    }    
    */
    cout<<"created template"<<endl;
    cv::imwrite("template_img.bmp", template_img);    
    
    //THE FOLLOWING IS FOR CREATING A  SYNTHETIC IMAGE, FOR TESTING ONLY
    //REPLACE THIS WITH REAL IMAGE, i.e. depthmap_image, as computed above 
    int hu= 5; //define dimensions of a rectangle; height corresponds to u axis
    int half_hu = hu/2;
    int wv= 7; //width of rectangle, corresonds to v-axis
    int half_wv = wv/2;
    int u_center = 11; //choose to center the pattern of interest at these coordinates in image
    int v_center = 14;
    //synthesize an image (instead of reading image)
    cv::Mat embedded_img(Nu,Nv, CV_8U, cv::Scalar(0)); //image, actual size;
    //put a rectangle in the src_img: hu x wv, centered at u_center,v_center
    for (u=u_center-half_hu;u<=u_center+half_hu;u++) {
        for (v=v_center-half_wv;v<=v_center+half_wv;v++) {
            cout<<"u="<<u<<"; v="<<v<<endl;
            embedded_img.at<uchar>(u, v) = 255;
        }
    }   
    cout<<"created false image"<<endl;
    cv::imwrite("embedded_img.bmp", embedded_img);    
    //END OF SYNTHESIZING TEST IMAGE
    
    //create a larger image with room for padding around the edges
    cv::Mat padded_img(Nu+2*Ntu-1, Nv+2*Ntv-2, CV_8U, cv::Scalar(0)); //create image, set encoding and size, init pixels to default val
    //make a checkerboard background; want to pad the image w/ unbiased background
    //start by making this entire image a checkerboard
    /* try w/o checkerboard
    for (u=0;u<Nu+2*Ntu-1;u+=2) {
        for (v=0;v<Nv+2*Ntv-1;v+=2) {
            padded_img.at<uchar>(u, v) = 255;
        }
    }
    */
    //now copy over the image to be embedded (centered) in checkerboard; edges will still be unbiased
    for (u=0;u<Nu;u++) {
        for (v=0;v<Nv;v++) {
            //leaves edges of halfwidth of template with unbiased (checkerboard) padding
            //padded_img.at<uchar>(u+Ntu-1, v+Ntv-1) = embedded_img.at<uchar>(u, v);
            padded_img.at<uchar>(u+Ntu-1, v+Ntv-1) = depthmap_image.at<uchar>(u, v); //use real image    
        }
    }    
    cout<<"padded image with checkerboard"<<endl;
    cv::imwrite("padded_img.bmp", padded_img);
    
    //now compute distance between template and snippets of image, and put results into another image
    cv::Mat result_mat;  //put result of template matching in this output matrix; 
    /*		// method: CV_TM_SQDIFF, CV_TM_SQDIFF_NORMED, CV_TM _CCORR, CV_TM_CCORR_NORMED, CV_TM_CCOEFF, CV_TM_CCOEFF_NORMED
		int match_method = CV_TM_CCORR_NORMED;
		cv::matchTemplate(src_img, template_img, result_mat, match_method);
		cv::normalize(result_mat, result_mat, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());*/
    int match_method = CV_TM_SQDIFF_NORMED; //CV_TM_SQDIFF
    cv::matchTemplate(padded_img, template_img, result_mat, match_method);
    //take sqrt of all match values:
    /*
    for (u=0;u<Nu+2*Ntu-2;u++) {
        for (v=0;v<Nv+2*Ntv-2;v++) {
            result_mat.at<uchar>(u, v) = sqrt(result_mat.at<uchar>(u, v));
        }
    }    
    */
    cout<<"result_mat: "<<endl<<result_mat<<endl;
    //cv::normalize(result_mat, result_mat); //, 0, 255); //, cv::NORM_MINMAX, -1, cv::Mat());
    cv::normalize(result_mat, result_mat, 0, 255, cv::NORM_MINMAX, -1, cv::Mat());

    cout<<"normalized result mat: "<<endl<<result_mat<<endl;
    cv::imwrite("result_mat.bmp", result_mat);
    
    //find location(s) of best template fits:
    double minVal, maxVal; 
    cv::Point minLoc, maxLoc, matchLoc;
    cv::minMaxLoc(result_mat, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat() );    
    cout<<"best match at: "<<minLoc<<endl;
    int x_fit = minLoc.x;
    int y_fit = minLoc.y;
    cout<<"x_fit = "<<x_fit<<"; y_fit = "<<y_fit<<endl;
    //erase this match and try again:
    for (int i=x_fit-Rpix+tu_c;i<=x_fit+Rpix+tu_c;i++) {
        for (int j=y_fit-Rpix+tv_c;j<=y_fit+Rpix+tv_c;j++) {
            padded_img.at<uchar>(i,j) = 0;
        }
    }
    cv::imwrite("padded_img2.bmp", padded_img);
    cv::Mat result_mat2;
    match_method = CV_TM_SQDIFF;
    
    cv::matchTemplate(padded_img, template_img, result_mat2, match_method);
    cout<<endl<<endl<<endl;
    cout<<"result_mat2: "<<endl<<result_mat2<<endl;    
    cv::minMaxLoc(result_mat2, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat() );    
    cout<<"2nd pass: best match at: "<<minLoc<<endl;      
    for (int i=minLoc.y-5;i<minLoc.y+7;i++) {
        cout<<"i: "<<i<<";  result(i,u_fit): "<<result_mat2.at<float>(i,minLoc.x)<<endl;
    }
    for (int i=minLoc.y-5;i<minLoc.y+7;i++) {
        cout<<"i: "<<i<<";  result(i,u_fit+1): "<<result_mat2.at<float>(i,minLoc.x+1)<<endl;
    }    
    cv::normalize(result_mat2, result_mat2, 0, 255, cv::NORM_MINMAX, -1, cv::Mat());

    //cout<<"normalized result mat: "<<endl<<result_mat<<endl;
    cv::imwrite("result_mat2.bmp", result_mat2);    
    cv::minMaxLoc(result_mat2, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat() );    
    //cout<<"2nd pass: best match at: "<<minLoc<<endl;    
    
    //cv::namedWindow( "Display_window" );// Create a window for display.
    //cv::imshow( "Display window", src_img );                   // Show our image inside it.

    //cv::waitKey(0);      
    //publish the point cloud in a ROS-compatible message; here's a publisher:
    ros::Publisher pubCloud = nh.advertise<sensor_msgs::PointCloud2> ("/pcd", 1);
    sensor_msgs::PointCloud2 ros_cloud; //here is the ROS-compatible message
    pcl::toROSMsg(*pcl_clr_ptr, ros_cloud); //convert from PCL to ROS type this way
    ros_cloud.header.frame_id = "depth_camera_1_frame";

    //publish the transformed cloud as well:
    ros::Publisher pubCloud2 = nh.advertise<sensor_msgs::PointCloud2> ("/transformed_cloud", 1);
    sensor_msgs::PointCloud2 ros_cloud2; //here is the ROS-compatible message
    pcl::toROSMsg(*output_cloud_ptr, ros_cloud2); //convert from PCL to ROS type this way
    ros_cloud2.header.frame_id = "bin_frame";

    cout << "view in rviz; choose: topic= pcd; and fixed frame= depth_camera_1_frame" << endl;
    cout << "view transformed image with: topic= /transformed_cloud; and fixed frame= bin_frame" << endl;

    //publish the ROS-type message on topic "/ellipse"; can view this in rviz
    while (ros::ok()) {

        pubCloud.publish(ros_cloud);
        pubCloud2.publish(ros_cloud2);

        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }
    return 0;
}

