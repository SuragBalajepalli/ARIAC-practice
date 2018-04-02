//box_inspector.cpp implementation of class/library
#include <box_inspector/box_inspector.h>
//#include "box_inspector_fncs.cpp" //more code, outside this file
#include "box_inspector_fncs.cpp" //more code, outside this file

BoxInspector::BoxInspector(ros::NodeHandle* nodehandle) : nh_(*nodehandle) { //constructor
  //set up camera subscriber:
   box_camera_subscriber_ = nh_.subscribe("/ariac/box_camera_1", 1, &BoxInspector::box_camera_callback, this);
   got_new_snapshot_=false; //trigger to get new snapshots

}

bool BoxInspector::compare_pose(osrf_gear::Model model_A, osrf_gear::Model model_B) {
 		Eigen::Affine3d affine1,affine2;
		Eigen::Vector3d origin_diff;
		affine1=xformUtils_.transformPoseToEigenAffine3d(model_A.pose);
		affine2=xformUtils_.transformPoseToEigenAffine3d(model_B.pose);
		origin_diff = affine1.translation()-affine2.translation();
		double origin_err = origin_diff.norm();
		Eigen::Matrix3d R1,R2,R_diff;
		R1 = affine1.linear();
		R2 = affine2.linear();
		R_diff = R1.inverse()*R2;
		Eigen::AngleAxisd angleAxis(R_diff);
		double rotation_err = angleAxis.angle();
		double rotation_err_corrected;
		rotation_err_corrected=2.0*M_PI-rotation_err;
		
    ROS_INFO("corrected_err= %f", rotation_err_corrected);
    ROS_INFO("rotation_err= %f", rotation_err);
  		if(origin_err<5 && rotation_err<1) {
  			return true;
  		}
 	else { return false; }
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
void BoxInspector::get_new_snapshot_from_box_cam() {
  got_new_snapshot_= false;
  ROS_INFO("waiting for snapshot from camera");
  while (!got_new_snapshot_) {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }
  ROS_INFO("got new snapshot");
}


//here is  the main fnc; provide a list of models, expressed as desired parts w/ poses w/rt box;
//get a box-camera logical image and  parse it
//populate the vectors as follows:
// satisfied_models_wrt_world: vector of models that match shipment specs, including precision location in box
// misplaced_models_wrt_world: vector of models that belong in the shipment, but are imprecisely located in box
// missing_models_wrt_world:  vector of models that are requested in the shipment, but not yet present in the box
// orphan_models_wrt_world: vector of models that are seen in the box, but DO NOT belong in the box
  void BoxInspector::update_inspection(vector<osrf_gear::Model> desired_models_wrt_world,
       vector<osrf_gear::Model> &satisfied_models_wrt_world,
       vector<osrf_gear::Model> &misplaced_models_actual_coords_wrt_world,
       vector<osrf_gear::Model> &misplaced_models_desired_coords_wrt_world,
       vector<osrf_gear::Model> &missing_models_wrt_world,
       vector<osrf_gear::Model> &orphan_models_wrt_world) {
  //WRITE ME!!!
  ROS_WARN("NEED TO WRITE update_inspection() ");
  got_new_snapshot_=false;
  while(!got_new_snapshot_) {
     ros::spinOnce(); // refresh camera image
     ros::Duration(0.5).sleep();
     ROS_INFO("waiting for logical camera image");
  }
  ROS_INFO("got new image");
  int num_parts_seen =  box_inspector_image_.models.size();
  ROS_INFO("update_inspection: box camera saw %d objects",num_parts_seen);
  int num_parts_desired = desired_models_wrt_world.size();
  orphan_models_wrt_world.clear(); //this will be empty, unless something very odd happens
  satisfied_models_wrt_world.clear();  //shipment will be complete when this matches parts/poses specified in shipment
  misplaced_models_actual_coords_wrt_world.clear();
  misplaced_models_desired_coords_wrt_world.clear();
  missing_models_wrt_world.clear();
  int num_each_parts_seen[5]={0,0,0,0,0};
  int num_each_parts_des[5]={0,0,0,0,0};
  for(int i = 0; i<num_parts_seen;i++) {
  	switch (mappings[box_inspector_image_.models[i].type]) {
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
  		switch (mappings[desired_models_wrt_world[i].type]) {
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
  //THIS  IS WRONG...but an example of how to get models from image and sort into category vectors
  	}

	for(int ipart=0;ipart<5;ipart++) {
        ROS_INFO("debug %d", ipart);

		if (num_each_parts_seen[ipart]<num_each_parts_des[ipart]) {
			for(int j=0;j<desired_models_wrt_world.size();j++) {
			if(desired_models_wrt_world[j].type == part_id_to_name_mappings[ipart]) {
      	bool found=false;
				for(int i=0; i<box_inspector_image_.models.size();i++) {
					if(box_inspector_image_.models[i].type == desired_models_wrt_world[j].type) {	
					bool pose_comparison = compare_pose(box_inspector_image_.models[i],desired_models_wrt_world[j]);
					if (pose_comparison) {
						found=true;
					}
	}
  }			
				if (!found) {
					missing_models_wrt_world.push_back(desired_models_wrt_world[j]);
				}
			}
		}
	
}
		else if(num_each_parts_seen[ipart]>num_each_parts_des[ipart]) {
      for(int i=0; i<box_inspector_image_.models.size();i++) {
      if(box_inspector_image_.models[i].type == part_id_to_name_mappings[ipart]) {

        bool found=false;
				for(int j=0;j<desired_models_wrt_world.size();j++) {
					if(box_inspector_image_.models[i].type == desired_models_wrt_world[j].type) {
					bool pose_comparison = compare_pose(box_inspector_image_.models[i],desired_models_wrt_world[j]);
					if (pose_comparison) {
						found=true;
					}
	}
  }			
				if (!found) {
					orphan_models_wrt_world.push_back(box_inspector_image_.models[i]);
				}
			}
		}
}	

		else if(num_each_parts_des[ipart]==num_each_parts_seen[ipart]) {
			for(int i=0; i<box_inspector_image_.models.size();i++) {
        if(box_inspector_image_.models[i].type == part_id_to_name_mappings[ipart]) {
          bool found=false;
            for(int j=0;j<desired_models_wrt_world.size();j++) {
              if(box_inspector_image_.models[i].type == desired_models_wrt_world[j].type) {
					     bool pose_comparison = compare_pose(box_inspector_image_.models[i],desired_models_wrt_world[j]);
          if (pose_comparison) {
            found=true;
					}
				}
      }

				if (!found) {

					misplaced_models_actual_coords_wrt_world.push_back(box_inspector_image_.models[i]);

				}

	}
}
		
	
      for(int j=0;j<desired_models_wrt_world.size();j++) {
      if(desired_models_wrt_world[j].type == part_id_to_name_mappings[ipart]) {
       bool found=false;
			for(int i=0;i<box_inspector_image_.models.size();i++){
			if(box_inspector_image_.models[i].type == desired_models_wrt_world[j].type) {
      bool pose_comparison = compare_pose(box_inspector_image_.models[i],desired_models_wrt_world[j]);

      if (pose_comparison) {
					found=true;
				}
	}
  }

				if (!found) {
              ROS_INFO("debug1: %d", j);

  			misplaced_models_desired_coords_wrt_world.push_back(desired_models_wrt_world[j]);				
			}
}
    }
  }


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
            //ROS_WARN("USE THIS INFO TO COMPUTE BOX POSE WRT WORLD AND  POPULATE box_pose_wrt_world");
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
