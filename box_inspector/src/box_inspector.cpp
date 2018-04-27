//box_inspector.cpp implementation of class/library
#include <box_inspector/box_inspector.h>
//#include "box_inspector_fncs.cpp" //more code, outside this file
#include "box_inspector_fncs.cpp" //more code, outside this file
#include <math.h>

BoxInspector::BoxInspector(ros::NodeHandle* nodehandle) : nh_(*nodehandle) { //constructor
  //set up camera subscriber:
   box_camera_subscriber_ = nh_.subscribe("/ariac/box_camera_1", 1, &BoxInspector::box_camera_callback, this);
   got_new_snapshot_=false; //trigger to get new snapshots

}


bool BoxInspector::find_orphan_parts(vector<osrf_gear::Model> desired_models_wrt_world, vector<osrf_gear::Model> &orphan_models) {

  vector<osrf_gear::Model> satisfied_models_wrt_world,misplaced_models_desired_coords_wrt_world,misplaced_models_actual_coords_wrt_world,missing_wrt_world;
  if(!update_inspection(desired_models_wrt_world,satisfied_models_wrt_world,misplaced_models_actual_coords_wrt_world,misplaced_models_desired_coords_wrt_world,missing_wrt_world,orphan_models)) {
    return 0;
  }
if(orphan_models.size()==0) {
  return 0;
}
return 1;
}


bool BoxInspector::find_missing_parts(vector<osrf_gear::Model> desired_models_wrt_world, vector<osrf_gear::Model> &missing_wrt_world) {
  vector<osrf_gear::Model> satisfied_models_wrt_world,misplaced_models_desired_coords_wrt_world,misplaced_models_actual_coords_wrt_world,orphan_models_wrt_world;
 if(!update_inspection(desired_models_wrt_world,satisfied_models_wrt_world,misplaced_models_actual_coords_wrt_world,misplaced_models_desired_coords_wrt_world,missing_wrt_world,orphan_models_wrt_world)) {
  return 0;
 }
if(missing_wrt_world.size()==0) {
  return 0;
}
return 1;
}

void BoxInspector::model_to_part(osrf_gear::Model model, inventory_msgs::Part &part, unsigned short int location) {
    part.name = model.type;
    part.pose.pose = model.pose;
    part.location = location; //by default
}

bool BoxInspector::get_observed_part_pose(inventory_msgs::Part place_part, inventory_msgs::Part &observed_part) {
  get_new_snapshot_from_box_cam();
  int winner=0;
  float max_ht=0;
  int debug;
  geometry_msgs::PoseStamped grasped_pose_wrt_wrld;
  for(int i=1;i<box_inspector_image_.models.size();i++) {
    
    grasped_pose_wrt_wrld=compute_stPose(box_inspector_image_.pose, box_inspector_image_.models[i].pose);
    if(grasped_pose_wrt_wrld.pose.position.z>max_ht) {
      max_ht=grasped_pose_wrt_wrld.pose.position.z;
      winner=i;
      
    }
    
  }
  grasped_pose_wrt_wrld=compute_stPose(box_inspector_image_.pose,box_inspector_image_.models[winner].pose);
  ROS_INFO_STREAM("winning model is:"<<grasped_pose_wrt_wrld);
 
  model_to_part(box_inspector_image_.models[winner],observed_part);
  observed_part.pose=grasped_pose_wrt_wrld;
return 1;
}

bool BoxInspector::post_dropoff_check(vector<osrf_gear::Model> desired_models_wrt_world,vector<osrf_gear::Model> &misplaced_models_desired_coords, vector<osrf_gear::Model> &misplaced_models_actual_coords) {
  vector<osrf_gear::Model> satisfied_models_wrt_world,missing_models_wrt_world,orphan_models_wrt_world;
  if(!update_inspection(desired_models_wrt_world, satisfied_models_wrt_world,misplaced_models_actual_coords,misplaced_models_desired_coords,missing_models_wrt_world,orphan_models_wrt_world)) {
    return 0;
  }
  
  if (misplaced_models_desired_coords.size()==0 || misplaced_models_actual_coords.size()==0) {
    return 0;
    
  }

  else {
      
    return 1;
  }
}

bool BoxInspector::pre_dropoff_check(inventory_msgs::Part part,osrf_gear::Model misplaced_model_actual_coords, osrf_gear::Model misplaced_model_desired_coords) {
  osrf_gear::Model model;
  vector<osrf_gear::Model> desired,satisfied_models_wrt_world,missing_models_wrt_world,orphan_models_wrt_world,misplaced_models_actual_coords,misplaced_models_desired_coords;
  //spart_to_model(part,model);
  model.type=part.name;
  model.pose=part.pose.pose;
  desired.clear();
  desired.push_back(model);
  if(!update_inspection(desired,satisfied_models_wrt_world,misplaced_models_actual_coords,misplaced_models_desired_coords,missing_models_wrt_world, orphan_models_wrt_world)) {
    return 0;
  }
  if(misplaced_models_desired_coords.size()==0) {
    ROS_INFO("pre drop off check good");
    return 0;
  }
  else{
    misplaced_model_desired_coords=misplaced_models_desired_coords[0];
    misplaced_model_actual_coords=misplaced_models_actual_coords[0];
    return 1;
  }

}

bool BoxInspector::compare_pose(geometry_msgs::Pose pose_A, geometry_msgs::Pose pose_B) {
 		Eigen::Affine3d affine1,affine2;
		Eigen::Vector3d origin_diff;
		affine1=xformUtils_.transformPoseToEigenAffine3d(pose_A);
		affine2=xformUtils_.transformPoseToEigenAffine3d(pose_B);
		origin_diff = affine1.translation()-affine2.translation();
		double origin_err = origin_diff.norm();
		Eigen::Matrix3d R1,R2,R_diff;
		R1 = affine1.linear();
		R2 = affine2.linear();
		R_diff = R1.inverse()*R2;
		Eigen::AngleAxisd angleAxis(R_diff);
		double rotation_err = angleAxis.angle();
		if(origin_err<0.01 && rotation_err<0.1) {
  			return true;
  		}
 	else { return false; }
}



bool BoxInspector::compare_pose(geometry_msgs::PoseStamped pose_stamped_A, geometry_msgs::PoseStamped pose_stamped_B) {
  geometry_msgs::Pose pose_A,pose_B;
  pose_A=pose_stamped_A.pose;
  pose_B=pose_stamped_B.pose;
  bool comparison=compare_pose(pose_A,pose_B);
  return comparison;
}


//to request a new snapshot, set need_new_snapshot_ = true, and make sure to give a ros::spinOnce()
void BoxInspector::box_camera_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg) {
    if (!got_new_snapshot_) {
        box_inspector_image_ = *image_msg;  //copy the current message to a member data var, i.e. freeze the snapshot
        got_new_snapshot_ =  true;
        ROS_INFO_STREAM("received box-camera image of: "<<box_inspector_image_<<endl);
        int n_models = box_inspector_image_.models.size();
        ROS_INFO("%d models seen ",n_models);
    }
}

//method to request a new snapshot from logical camera; blocks until snapshot is ready,
// then result will be in box_inspector_image_
bool BoxInspector::get_new_snapshot_from_box_cam() {
  got_new_snapshot_= false;
  double dt= 0.05;
  double timer = 0.0;
  while (!got_new_snapshot_&&(timer<BOX_INSPECTOR_TIMEOUT)) {
        ros::spinOnce();
        ros::Duration(dt).sleep();
        timer+=dt;
    }
    if (timer>=BOX_INSPECTOR_TIMEOUT) {
        ROS_WARN("could not update box inspection image!");
        return false;
    }
    else {
      return true;
    }
  }


//here is  the main fnc; provide a list of models, expressed as desired parts w/ poses w/rt box;
//get a box-camera logical image and  parse it
//populate the vectors as follows:
// satisfied_models_wrt_world: vector of models that match shipment specs, including precision location in box
// misplaced_models_wrt_world: vector of models that belong in the shipment, but are imprecisely located in box
// missing_models_wrt_world:  vector of models that are requested in the shipment, but not yet present in the box
// orphan_models_wrt_world: vector of models that are seen in the box, but DO NOT belong in the box
bool BoxInspector::update_inspection(vector<osrf_gear::Model> desired_models_wrt_world,
       vector<osrf_gear::Model> &satisfied_models_wrt_world,
       vector<osrf_gear::Model> &misplaced_models_actual_coords_wrt_world,
       vector<osrf_gear::Model> &misplaced_models_desired_coords_wrt_world,
       vector<osrf_gear::Model> &missing_models_wrt_world,
       vector<osrf_gear::Model> &orphan_models_wrt_world) {
	int ans; // FOR DEBUG
  got_new_snapshot_=false;
  vector<geometry_msgs::Pose> sum_pose /* BAD VARIABLE NAME. FIX IT*/, average_pose;
  osrf_gear::LogicalCameraImage filtered_box_camera_image;
  filtered_box_camera_image = box_inspector_image_;
  ROS_INFO("Success 0"); 
  if(get_new_snapshot_from_box_cam()) {
  int num_parts_seen =  box_inspector_image_.models.size();
  ROS_INFO("update_inspection: box camera saw %d objects",num_parts_seen);
  sum_pose.resize(num_parts_seen);
  average_pose.resize(num_parts_seen);
  for (int i=0; i<num_parts_seen;i++) {
  	sum_pose[i].position.x = 0;
  	sum_pose[i].position.y = 0;
  	sum_pose[i].position.z = 0;
  	sum_pose[i].orientation.x = 0;
  	sum_pose[i].orientation.y = 0;
  	sum_pose[i].orientation.z = 0;
  	sum_pose[i].orientation.w = 0;
  	ROS_INFO("Success 1");
  }
  for (int i = 0; i < 3; i++) { // Should remove hard coded numbers and use while
  if(get_new_snapshot_from_box_cam()) {	
  	filtered_box_camera_image = box_inspector_image_;
  	if(box_inspector_image_.models.size() != num_parts_seen) {
  		break;
  		ROS_INFO("FAIl 1");
  	}
  	for(int j = 0; j < num_parts_seen; j++) {
  		sum_pose[j].position.x += box_inspector_image_.models[j].pose.position.x;
  		sum_pose[j].position.y += box_inspector_image_.models[j].pose.position.y;
  		sum_pose[j].position.z += box_inspector_image_.models[j].pose.position.z;
  		sum_pose[j].orientation.x += box_inspector_image_.models[j].pose.orientation.x;
  		sum_pose[j].orientation.y += box_inspector_image_.models[j].pose.orientation.y;
  		sum_pose[j].orientation.z += box_inspector_image_.models[j].pose.orientation.z;
  		sum_pose[j].orientation.w += box_inspector_image_.models[j].pose.orientation.w;
  		ROS_INFO("Success 2");
  	}
  }
}
  for (int j = 0; j < num_parts_seen; j++) {
	average_pose[j].position.x  = sum_pose[j].position.x/3;
	average_pose[j].position.y  = sum_pose[j].position.y/3;
	average_pose[j].position.z  = sum_pose[j].position.z/3;	
  	average_pose[j].orientation.x = sum_pose[j].orientation.x/3;
  	average_pose[j].orientation.y = sum_pose[j].orientation.y/3;
  	average_pose[j].orientation.z = sum_pose[j].orientation.z/3;
  	average_pose[j].orientation.w = sum_pose[j].orientation.w/3;
  	double n = sqrt(pow(average_pose[j].orientation.x,2.0) + pow(average_pose[j].orientation.y,2.0) + pow(average_pose[j].orientation.z,2.0) + pow(average_pose[j].orientation.w,2.0)); //SHOULD AVOID PERFORMANCE OVERHEAD OF MULTIPLE FNC CALL; change to x*x instead
  	average_pose[j].orientation.x /= n;
  	average_pose[j].orientation.y /= n;
  	average_pose[j].orientation.z /= n;
  	average_pose[j].orientation.w /= n;
  	ROS_INFO("Success 3");
  }

  for (int i = 0; i < num_parts_seen; i++) {
  	//filtered_box_camera_image.models[i].pose = average_pose[i];
  	ROS_INFO("Success 4");
  }

  ROS_INFO_STREAM("filtered box camera image"<< filtered_box_camera_image);
  cin>>ans;

  int num_parts_desired = desired_models_wrt_world.size();
  orphan_models_wrt_world.clear(); //this will be empty, unless something very odd happens
  satisfied_models_wrt_world.clear();  //shipment will be complete when this matches parts/poses specified in shipment
  misplaced_models_actual_coords_wrt_world.clear();
  misplaced_models_desired_coords_wrt_world.clear();
  missing_models_wrt_world.clear();
  int num_each_parts_seen[5]={0,0,0,0,0};
  int num_each_parts_des[5]={0,0,0,0,0};
  for(int i = 0; i<num_parts_seen;i++) {
  	switch (name_to_part_id_mappings[box_inspector_image_.models[i].type]) {
  		case 1:
  			num_each_parts_seen[0]+=1;
  			break;
  		case 2:
  			num_each_parts_seen[1]+=1;
  			break;
  		case 3:
  			num_each_parts_seen[2]+=1;
  			break;
  		case 4:
  			num_each_parts_seen[3]+=1;
  			break;
  		case 5:
  			num_each_parts_seen[4]+=1;
  			break;
  	}
  }
	for(int i = 0; i<num_parts_desired;i++) {
  		switch (name_to_part_id_mappings[desired_models_wrt_world[i].type]) {
  			case 1:
  				num_each_parts_des[0]+=1;
  				break;
  			case 2:
  				num_each_parts_des[1]+=1;
  				break;
  			case 3:
  				num_each_parts_des[2]+=1;
  				break;
  			case 4:
  				num_each_parts_des[3]+=1;
  				break;
  			case 5:
  				num_each_parts_des[4]+=1;
  				break;
  		}

  	}

  	for(int i=0;i<box_inspector_image_.models.size();i++) {
    	if(box_inspector_image_.models[i].type != "shipping_box"){
    	bool found=false;
    	for(int j=0;j<desired_models_wrt_world.size();j++) {
    		if(box_inspector_image_.models[i].type==desired_models_wrt_world[j].type) {
    			geometry_msgs::PoseStamped model_pose_from_image_wrt_world=compute_stPose(box_inspector_image_.pose, box_inspector_image_.models[i].pose);
				bool pose_comparison = compare_pose(model_pose_from_image_wrt_world.pose,desired_models_wrt_world[j].pose);
				if (pose_comparison) {
				found =true;
				}
    		}
    	}
    	if(found) {
    		geometry_msgs::PoseStamped pose_wrt_world =compute_stPose(box_inspector_image_.pose, box_inspector_image_.models[i].pose);
          osrf_gear::Model model;
          model=box_inspector_image_.models[i];
          model.pose=pose_wrt_world.pose;  
    		satisfied_models_wrt_world.push_back(model);
    	}
    }
}

	for(int ipart=0;ipart<5;ipart++) {
			

		if (num_each_parts_seen[ipart]<num_each_parts_des[ipart]) {
			for(int j=0;j<desired_models_wrt_world.size();j++) {
			if(desired_models_wrt_world[j].type == part_id_to_name_mappings[ipart+1]) {
      	bool found=false;
				for(int i=0; i<box_inspector_image_.models.size();i++) {
					if(box_inspector_image_.models[i].type == desired_models_wrt_world[j].type) {	
					geometry_msgs::PoseStamped model_pose_from_image_wrt_world=compute_stPose(box_inspector_image_.pose, box_inspector_image_.models[i].pose);
					bool pose_comparison = compare_pose(model_pose_from_image_wrt_world.pose,desired_models_wrt_world[j].pose);
					if (pose_comparison) {
						found=true;
					}
	}
  }			
				if (!found) {
					ROS_INFO("found missing");
					missing_models_wrt_world.push_back(desired_models_wrt_world[j]);
				}
			}
		}
	
}
		else if(num_each_parts_seen[ipart]>num_each_parts_des[ipart]) {
      for(int i=0; i<box_inspector_image_.models.size();i++) {

      if(box_inspector_image_.models[i].type == part_id_to_name_mappings[ipart+1]) {
      

        bool found=false;
				for(int j=0;j<desired_models_wrt_world.size();j++) {
					if(box_inspector_image_.models[i].type == desired_models_wrt_world[j].type) {
					geometry_msgs::PoseStamped model_pose_from_image_wrt_world=compute_stPose(box_inspector_image_.pose, box_inspector_image_.models[i].pose);
					bool pose_comparison = compare_pose(model_pose_from_image_wrt_world.pose,desired_models_wrt_world[j].pose);
					if (pose_comparison) {
						found=true;
					}
	}
  }			
				if (!found) {
          geometry_msgs::PoseStamped pose_wrt_world =compute_stPose(box_inspector_image_.pose, box_inspector_image_.models[i].pose);
          osrf_gear::Model model;
          model=box_inspector_image_.models[i];
          model.pose=pose_wrt_world.pose;  
  				orphan_models_wrt_world.push_back(model);
				}

			}
		}
}	

		else if(num_each_parts_des[ipart]==num_each_parts_seen[ipart]) {
			for(int i=0; i<box_inspector_image_.models.size();i++) {
        if(box_inspector_image_.models[i].type == part_id_to_name_mappings[ipart+1]) {
            bool found=false;
            for(int j=0;j<desired_models_wrt_world.size();j++) {
              if(box_inspector_image_.models[i].type == desired_models_wrt_world[j].type) {
					geometry_msgs::PoseStamped model_pose_from_image_wrt_world=compute_stPose(box_inspector_image_.pose, box_inspector_image_.models[i].pose);
					bool pose_comparison = compare_pose(model_pose_from_image_wrt_world.pose,desired_models_wrt_world[j].pose);
          if (pose_comparison) {
            found=true;
					}
				}
      }

				if (!found) {
					geometry_msgs::PoseStamped pose_wrt_world =compute_stPose(box_inspector_image_.pose, box_inspector_image_.models[i].pose);
          osrf_gear::Model model;
          model=box_inspector_image_.models[i];
          model.pose=pose_wrt_world.pose;  
					misplaced_models_actual_coords_wrt_world.push_back(model);
				}
				

	}
}
		
	
      for(int j=0;j<desired_models_wrt_world.size();j++) {
      if(desired_models_wrt_world[j].type == part_id_to_name_mappings[ipart+1]) {
       bool found=false;
			for(int i=0;i<box_inspector_image_.models.size();i++){
			if(box_inspector_image_.models[i].type == desired_models_wrt_world[j].type) {
      			geometry_msgs::PoseStamped model_pose_from_image_wrt_world=compute_stPose(box_inspector_image_.pose, box_inspector_image_.models[i].pose);
				bool pose_comparison = compare_pose(model_pose_from_image_wrt_world.pose,desired_models_wrt_world[j].pose);

      			if (pose_comparison) {
					found=true;
				}
	}
  }

				if (!found) {

  			misplaced_models_desired_coords_wrt_world.push_back(desired_models_wrt_world[j]);				
			}
}	
    }
    

    }

  }
  return 1;
}
else {
  return 0;
}

}






//intent of this function is, get a snapshot from the box-inspection camera;
//parse the image data to see if a shipping box is present
//if not, return false
//if seen, transform box pose to box pose w/rt world frame,
//    copy data to reference arg box_pose_wrt_world
//    and return "true"

bool BoxInspector::get_box_pose_wrt_world(geometry_msgs::PoseStamped &box_pose_wrt_world) {
    geometry_msgs::Pose cam_pose, box_pose; //cam_pose is w/rt world, but box_pose is w/rt camera

    //get a new snapshot of the box-inspection camera:
    get_new_snapshot_from_box_cam();

    //ROS_INFO("got box-inspection camera snapshot");
    //look for box in model list:
    int num_models = box_inspector_image_.models.size(); //how many models did the camera see?
    if (num_models == 0) return false;
    string box_name("shipping_box"); //does a model match this name?
    osrf_gear::Model model;
    cam_pose = box_inspector_image_.pose;
    ROS_INFO("box cam sees %d models", num_models);
    for (int imodel = 0; imodel < num_models; imodel++) {
        model = box_inspector_image_.models[imodel];
        string model_name(model.type);
        if (model_name == box_name) {
            box_pose = model.pose;
            ROS_INFO_STREAM("get_box_pose_wrt_world(): found box at pose " << box_pose << endl);
            
            box_pose_wrt_world = compute_stPose(cam_pose, box_pose);
            ROS_INFO_STREAM("box_pose_wrt_world: " << box_pose_wrt_world << endl);
            return true;
        }
    }
    //if reach here, did not find shipping_box
    ROS_WARN("get_box_pose_wrt_world(): shipping-box not seen!");
    return false;
}

//helper  function:
//given a camera pose and a part-pose (or box-pose) w/rt camera, compute part pose w/rt world
//xform_utils library should help here

geometry_msgs::PoseStamped BoxInspector::compute_stPose(geometry_msgs::Pose cam_pose, geometry_msgs::Pose part_pose) {

    geometry_msgs::PoseStamped stPose_part_wrt_world;
    //compute part-pose w/rt world and return as a pose-stamped message object
    Eigen::Affine3d cam_wrt_world, part_wrt_cam, part_wrt_world;
    
    cam_wrt_world = xformUtils_.transformPoseToEigenAffine3d(cam_pose);
    part_wrt_cam = xformUtils_.transformPoseToEigenAffine3d(part_pose);
    part_wrt_world = cam_wrt_world*part_wrt_cam;
    geometry_msgs::Pose pose_part_wrt_world = xformUtils_.transformEigenAffine3dToPose(part_wrt_world);
    geometry_msgs::PoseStamped part_pose_stamped;
    part_pose_stamped.header.stamp = ros::Time::now();
    part_pose_stamped.header.frame_id = "world";
    part_pose_stamped.pose = pose_part_wrt_world;
    return part_pose_stamped;
}


//rosmsg show osrf_gear/LogicalCameraImage: 
/*
osrf_gear/Model[] models
  string type
  geometry_msgs/Pose pose
    geometry_msgs/Point position
      float64 x
      float64 y
      float64 z
    geometry_msgs/Quaternion orientation
      float64 x
      float64 y
      float64 z
      float64 w
geometry_msgs/Pose pose
  geometry_msgs/Point position
    float64 x
    float64 y
    float64 z
  geometry_msgs/Quaternion orientation
    float64 x
    float64 y
    float64 z
    float64 w
*/
