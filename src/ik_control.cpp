#include "ik_control.h"
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <thread>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <ros/package.h>

#include <shape_msgs/Mesh.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/mesh_operations.h>

using namespace dual_manipulation::ik_control;

ikControl::ikControl():db_mapper_(/*"test.db"*/)
{
    busy["left_hand"]=false;
    busy["right_hand"]=false;
    busy["both_hands"]=false;

    hand_pub["exec"]["left_hand"] = node.advertise<std_msgs::String>("/ik_control/left_hand/action_done",1,this);
    hand_pub["exec"]["right_hand"] = node.advertise<std_msgs::String>("/ik_control/right_hand/action_done",1,this);
    hand_pub["exec"]["both_hands"] = node.advertise<std_msgs::String>("/ik_control/both_hands/action_done",1,this);
    hand_pub["plan"]["left_hand"] = node.advertise<std_msgs::String>("/ik_control/left_hand/planning_done",1,this);
    hand_pub["plan"]["right_hand"] = node.advertise<std_msgs::String>("/ik_control/right_hand/planning_done",1,this);
    hand_pub["plan"]["both_hands"] = node.advertise<std_msgs::String>("/ik_control/both_hands/planning_done",1,this);
    hand_pub["check"]["left_hand"] = node.advertise<std_msgs::String>("/ik_control/left_hand/check_done",1,this);
    hand_pub["check"]["right_hand"] = node.advertise<std_msgs::String>("/ik_control/right_hand/check_done",1,this);
    hand_pub["check"]["both_hands"] = node.advertise<std_msgs::String>("/ik_control/both_hands/check_done",1,this);
    hand_pub["grasp"]["left_hand"] = node.advertise<std_msgs::String>("/ik_control/left_hand/grasp_done",1,this);
    hand_pub["grasp"]["right_hand"] = node.advertise<std_msgs::String>("/ik_control/right_hand/grasp_done",1,this);
    
    traj_pub_["left_hand"] = node.advertise<trajectory_msgs::JointTrajectory>("/left_arm/joint_trajectory_controller/command",1,this);
    traj_pub_["right_hand"] = node.advertise<trajectory_msgs::JointTrajectory>("/right_arm/joint_trajectory_controller/command",1,this);
    
    hand_synergy_pub_["left_hand"] = node.advertise<trajectory_msgs::JointTrajectory>("/left_hand/joint_trajectory_controller/command",1,this);
    hand_synergy_pub_["right_hand"] = node.advertise<trajectory_msgs::JointTrajectory>("/right_hand/joint_trajectory_controller/command",1,this);
    
    robot_state_publisher_ = node.advertise<moveit_msgs::DisplayRobotState>( "/ik_control/robot_state", 1 );
    
    kinematics_plugin_["left_hand"] = new kdl_kinematics_plugin::KDLKinematicsPlugin();
    kinematics_plugin_["right_hand"] = new kdl_kinematics_plugin::KDLKinematicsPlugin();
    
    group_map_["left_hand"] = "left_hand_arm";
    group_map_["right_hand"] = "right_hand_arm";
    group_map_["both_hands"] = "full_robot";
    
    moveGroups_["left_hand"] = new move_group_interface::MoveGroup(group_map_.at("left_hand"));
    moveGroups_["right_hand"] = new move_group_interface::MoveGroup(group_map_.at("right_hand"));
    moveGroups_["both_hands"] = new move_group_interface::MoveGroup(group_map_.at("both_hands"));
    
    ee_map_["left_hand"] = moveGroups_.at("left_hand")->getEndEffectorLink();
    ee_map_["right_hand"] = moveGroups_.at("right_hand")->getEndEffectorLink();
    
    // unconmment to set a different tolerance (to 0.005 m / 0.005 rad = 0.5 degree in this case)
    for(auto item:moveGroups_)
    {
      item.second->setPlannerId("RRTConnectkConfigDefault");
      item.second->setGoalPositionTolerance(0.005);
      item.second->setGoalOrientationTolerance(0.005);
      item.second->setWorkspace(-1.2,-1.5,-1.5,0.4,1.5,1.5);
    }

    movePlans_["left_hand"];
    movePlans_["right_hand"];
    movePlans_["both_hands"];
    
    // NOTE: attempted value of search_discretization: it's not clear what it is used for
    kinematics_plugin_.at("left_hand")->initialize("robot_description",group_map_.at("left_hand"),"world",ee_map_.at("left_hand"),0.005);
    kinematics_plugin_.at("right_hand")->initialize("robot_description",group_map_.at("right_hand"),"world",ee_map_.at("right_hand"),0.005);
    
    isInitialized_ = true;
    
    // publishers for objects in the scene
    attached_collision_object_publisher_ = node.advertise<moveit_msgs::AttachedCollisionObject>("/attached_collision_object",1);
    collision_object_publisher_ = node.advertise<moveit_msgs::CollisionObject>("/collision_object",1);
    
    // give me all robot links in order to set allowed collisions map
    std::vector<std::string> links = moveGroups_.at("left_hand")->getCurrentState()->getRobotModel()->getLinkModelNamesWithCollisionGeometry();
    
    std::string left_hand="left_hand";
    std::string right_hand="right_hand";

    allowed_collisions_[left_hand].push_back("left_arm_7_link");
    allowed_collisions_[right_hand].push_back("right_arm_7_link");
    for (auto item:links)
      if (item.compare(0,left_hand.size(),left_hand.c_str()) == 0)
	allowed_collisions_[left_hand].push_back(item);
      else if (item.compare(0,right_hand.size(),right_hand.c_str()) == 0)
	allowed_collisions_[right_hand].push_back(item);
      
    // // quick check
    // for (auto item:allowed_collisions_)
    // {
    //   std::cout << item.first << ": " << std::endl;
    //   for (auto item2:item.second)
    //     std::cout << '\t' << item2 << std::endl;
    // }
    
    // check if the object DB is loaded correctly
    std::cout << "Object DB:" << std::endl;
    for(auto item:db_mapper_.Objects)
      std::cout << " - " << item.first << ": " << std::get<0>(item.second) << " + " << std::get<1>(item.second) << std::endl;
    
}

bool ikControl::manage_object(dual_manipulation_shared::scene_object_service::Request& req)
{
  if (req.command == "add")
  {
    return addObject(req);
  }
  else if (req.command == "remove")
  {
    return removeObject(req.object_id);
  }
  else if (req.command == "attach")
  {
    return attachObject(req);
  }
  else if (req.command == "detach")
  {
    return addObject(req);
  }
  else
  {
    ROS_ERROR("IKControl::manage_object: Unknown command %s, returning",req.command.c_str());
    return false;
  }
}

void ikControl::loadAndAttachMesh(moveit_msgs::AttachedCollisionObject &attObject)
{
  // load the appropriate mesh and add it to the CollisionObject
  
  // std::string path = ros::package::getPath("dual_manipulation_grasp_db");
  // path.append("/object_meshes/");
  
  // NOTE: the mesh should be in ASCII format, in meters, and the frame of reference should be coherent with the external information we get (obviously...)
  shapes::Mesh* m;
  shape_msgs::Mesh co_mesh;
  shapes::ShapeMsg co_mesh_msg;
  
  // m = shapes::createMeshFromResource("file://" + path + std::get<1>(db_mapper_.Objects.at(1)));
  m = shapes::createMeshFromResource("package://dual_manipulation_grasp_db/object_meshes/" + std::get<1>(db_mapper_.Objects.at( (int)attObject.weight )));
  m->scale(/*0.00*/1); // change this to 0.001 if expressed in mm; this does not change the frame, thus if it's not baricentric the object will be moved around
  shapes::constructMsgFromShape(m,co_mesh_msg);
  co_mesh = boost::get<shape_msgs::Mesh>(co_mesh_msg);
  
  // std::cout << "Attached Collision Object:" << std::endl;
  // std::cout << attObject << std::endl;

  attObject.object.meshes.push_back(co_mesh); //.push_back(ros: path);
}

bool ikControl::addObject(dual_manipulation_shared::scene_object_service::Request req)
{
  req.attObject.object.operation = req.attObject.object.ADD;

  moveit_msgs::AttachedCollisionObject attObject = req.attObject;
  
  loadAndAttachMesh(attObject);
  
  ROS_INFO_STREAM("Putting the object " << attObject.object.id << " into the environment (" << req.attObject.object.header.frame_id << ")...");

  if ((req.command == "add") && (grasped_objects_map_.count(attObject.object.id)))
  {
    ROS_WARN_STREAM("The object was already attached to " << grasped_objects_map_.at(attObject.object.id).link_name << ": moving it to the environment");
    removeObject(attObject.object.id);
  }
  else if (req.command == "detach")
  {
    if (grasped_objects_map_.count(attObject.object.id))
    {
      removeObject(attObject.object.id);
    }
    else
    {
      ROS_WARN_STREAM("The object to detach (" << req.attObject.object.id << ") was not attached to any end-effector: did you grasp it before?");
      return false;
    }
  }
  
  collision_object_publisher_.publish(attObject.object);
  
  // store information about this object in a class map
  world_objects_map_[attObject.object.id] = attObject;
  
  return true;
}

bool ikControl::removeObject(std::string& object_id)
{
  // remove associated information about this object
  if (!(world_objects_map_.count(object_id) || grasped_objects_map_.count(object_id) ))
  {
    ROS_WARN_STREAM("The object " << object_id << " was not in the scene!");
    return false;
  }
  else
  {
    std::string where;
    
    if (world_objects_map_.count(object_id))
    {
      where = world_objects_map_.at(object_id).object.header.frame_id;
      /* Define the message and publish the operation*/
      moveit_msgs::CollisionObject remove_object;
      remove_object.id = object_id;
      remove_object.header.frame_id = world_objects_map_.at(object_id).object.header.frame_id;
      remove_object.operation = remove_object.REMOVE;
      collision_object_publisher_.publish(remove_object);
      
      // erase the object from the map
      world_objects_map_.erase( world_objects_map_.find(object_id) );
    }
    else if (grasped_objects_map_.count(object_id))
    {
      where = grasped_objects_map_.at(object_id).link_name;
      // NOTE: this does not put the object back in the world
      moveit_msgs::AttachedCollisionObject detach_object;
      detach_object.object.id = object_id;
      detach_object.link_name = grasped_objects_map_.at(object_id).link_name;
      detach_object.object.operation = detach_object.object.REMOVE;
      attached_collision_object_publisher_.publish(detach_object);
      
      // erase the object from the map
      grasped_objects_map_.erase( grasped_objects_map_.find(object_id) );
    }
    
    ROS_INFO_STREAM("IKControl::removeObject: Object " << object_id << " removed from " << where);
  }
  
  return true;
}

bool ikControl::attachObject(dual_manipulation_shared::scene_object_service::Request& req)
{
  req.attObject.object.operation = req.attObject.object.ADD;
  
  moveit_msgs::AttachedCollisionObject& attObject = req.attObject;
  
  loadAndAttachMesh(attObject);

  ROS_INFO("IKControl::attachObject: attaching the object %s to %s",attObject.object.id.c_str(),attObject.link_name.c_str());
  // remove associated information about this object
  if ((!world_objects_map_.count(attObject.object.id)) && (!grasped_objects_map_.count(attObject.object.id)))
  {
    ROS_WARN_STREAM("The object " << attObject.object.id << " is not in the scene! Attaching it to " << attObject.link_name << " anyway...");
  }
  else
  {
    // remove the object from where it is currently
    removeObject(attObject.object.id);
  }
  
  // attach it to the robot
  attached_collision_object_publisher_.publish(attObject);
  
  // store information about this object in a class map
  grasped_objects_map_[attObject.object.id] = attObject;
  
  return true;
}

void ikControl::ik_check_thread(dual_manipulation_shared::ik_service::Request req)
{
  ROS_INFO("IKControl::ik_check_thread: Thread spawned! Computing IK for %s",req.ee_name.c_str());
  
  kdl_kinematics_plugin::KDLKinematicsPlugin* kin_ptr = kinematics_plugin_.at(req.ee_name.c_str());
  
  std::vector <double> ik_seed_state;
  std::vector <double> solution;
  moveit_msgs::MoveItErrorCodes error_code;
  
  // ik_seed_state.resize(kin_ptr->getJointNames().size());
  // for (auto item:ik_seed_state)
  //   item = 0.0;
  
  // use current joint values as initial guess
  ik_seed_state = moveGroups_.at(req.ee_name)->getCurrentJointValues();
  
  kin_ptr->getPositionIK(req.ee_pose.at(0), ik_seed_state, solution, error_code);
  
//   // alternate way...which doesn't work
//   if (moveGroups_.at(req.ee_name)->setJointValueTarget(req.ee_pose.at(0)))
//     error_code.val = 1;
//   else
//     error_code.val = 9999;
//   std::vector<std::string> joints = moveGroups_.at(req.ee_name)->getJoints();
// 
//   for(auto item:joints)
//   {
//     solution.push_back(* moveGroups_.at(req.ee_name)->getJointValueTarget().getJointPositions(item));
//   }
  
  if (error_code.val == 1)
      ROS_INFO_STREAM("IKControl::ik_check_thread: error_code.val = " << error_code.val << std::endl);
  else
      ROS_WARN_STREAM("IKControl::ik_check_thread: error_code.val = " << error_code.val << std::endl);
//   std::vector<std::string> joint_names = moveGroups_.at(req.ee_name)->getJoints();
//   for (auto item:joint_names)
//     std::cout << item << " | ";
//   std::cout << std::endl;
  for (auto item:solution)
    std::cout << item << " | ";
  std::cout << std::endl;
  
  if(error_code.val == 1)
  {
    msg.data = "done";
  }
  else
  {
    msg.data = "error";
  }
  hand_pub.at("check").at(req.ee_name).publish(msg); //publish on a topic when the IK check is done

  busy.at(req.ee_name)=false;
  
  return;
}

void ikControl::planning_thread(dual_manipulation_shared::ik_service::Request req)
{
  
    ROS_INFO("IKControl::planning_thread: Thread spawned! Computing plan for %s",req.ee_name.c_str());
    
    // this connecs to a running instance of the move_group node
    move_group_interface::MoveGroup* localMoveGroup = moveGroups_.at(req.ee_name);
    
    std::cout << "IKControl::planning_thread: Planning for group " << group_map_.at(req.ee_name) << std::endl;
//     geometry_msgs::Pose current_pose = localMoveGroup->getCurrentPose().pose;
//     
//     std::cout << "pos [x y z]: " << current_pose.position.x << " " << current_pose.position.y << " " << current_pose.position.z << std::endl;
//     std::cout << "orient [x y z w]: "  << current_pose.orientation.x << " " << current_pose.orientation.y << " " << current_pose.orientation.z << " " << current_pose.orientation.w << std::endl;

    bool target_set = false;
    if (req.ee_name != "both_hands")
    {
      target_set = localMoveGroup->setPoseTarget(req.ee_pose.at(0));
    }
    else
    {
      // // a workaround to move both arms and avoid collisions, as the simple following line doesn't work... it only moves the left hand
      // target_set = localMoveGroup->setPoseTarget(req.ee_pose.at(1),"right_hand_palm_link") && localMoveGroup->setPoseTarget(req.ee_pose.at(0),"left_hand_palm_link");
      
      moveit::planning_interface::MoveGroup::Plan tmpPlan;
      std::vector<double> left_joints, right_joints;
      
      target_set = moveGroups_.at("left_hand")->setPoseTarget(req.ee_pose.at(0));
      
      if(target_set)
      {
	moveGroups_.at("left_hand")->plan(tmpPlan);
	left_joints = tmpPlan.trajectory_.joint_trajectory.points.back().positions;
	
	std::cout << "left_joints = ";
	for(auto item:left_joints)
	  std::cout << item << "|";
	std::cout << std::endl;
      }
      
      target_set = target_set && moveGroups_.at("right_hand")->setPoseTarget(req.ee_pose.at(1));
      if (target_set)
      {
	moveit::core::RobotStatePtr current_state = moveGroups_.at("right_hand")->getCurrentState();
	current_state->setJointGroupPositions("left_hand_arm",left_joints);
	
	// const moveit::core::JointModelGroup *jmg = current_state->getJointModelGroup("right_hand_arm");
	// for(auto item:jmg->getJointModelNames())
	//   std::cout << item << " | ";
	// std::cout << std::endl;
	
	moveGroups_.at("right_hand")->plan(tmpPlan);
	right_joints = tmpPlan.trajectory_.joint_trajectory.points.back().positions;
	
	std::cout << "right_joints = ";
	for(auto item:right_joints)
	  std::cout << item << "|";
	std::cout << std::endl;
      }

      std::vector<double> bimanual_joints;
      bimanual_joints.resize(localMoveGroup->getCurrentJointValues().size() - left_joints.size() - right_joints.size());

      bimanual_joints.insert(bimanual_joints.end(),left_joints.begin(),left_joints.end());
      bimanual_joints.insert(bimanual_joints.end(),right_joints.begin(),right_joints.end());

      target_set = target_set && localMoveGroup->setJointValueTarget( bimanual_joints );
    }
    
    if ( target_set )
    {
      ROS_INFO_STREAM("IKControl::planning_thread: Target set correctly!" << std::endl);
    }
    else
    {
      ROS_WARN_STREAM("IKControl::planning_thread: Unable to set target pose\n");
      msg.data = "error";
      hand_pub.at("plan").at(req.ee_name).publish(msg); //publish on a topic when the trajectory is done

      busy.at(req.ee_name)=false;

      return;
    }
    
    moveit::planning_interface::MoveGroup::Plan* movePlan = &(movePlans_.at(req.ee_name));
    
    localMoveGroup->setStartStateToCurrentState();
    moveit::planning_interface::MoveItErrorCode error_code = localMoveGroup->plan(*movePlan);
    
    ROS_INFO_STREAM("movePlan traj size: " << movePlan->trajectory_.joint_trajectory.points.size() << std::endl);
    for (int i=0; i<movePlan->trajectory_.joint_trajectory.points.size(); ++i)
    {
      ROS_DEBUG_STREAM(movePlan->trajectory_.joint_trajectory.points.at(i) << std::endl);
      // std::cout << movePlan->trajectory_.joint_trajectory.points.at(i) << std::endl;
    }
    
    ROS_DEBUG_STREAM("pos [x y z]: " << req.ee_pose.at(0).position.x << " " << req.ee_pose.at(0).position.y << " " << req.ee_pose.at(0).position.z << std::endl);
    ROS_DEBUG_STREAM("orient [x y z w]: "  << req.ee_pose.at(0).orientation.x << " " << req.ee_pose.at(0).orientation.y << " " << req.ee_pose.at(0).orientation.z << " " << req.ee_pose.at(0).orientation.w << std::endl);

    // /* get a robot state message describing the pose in robot_state_ */
    // moveit_msgs::DisplayRobotState robotStateMsg;
    // robot_state::robotStateToRobotStateMsg(*robot_state_, robotStateMsg.state);
    // /* send the message to the RobotState display */
    // robot_state_publisher_.publish( robotStateMsg );

    /* let ROS send the message */
    ros::spinOnce();

    if (error_code.val == 1)
    {
      msg.data = "done";
    }
    else
    {
      msg.data = "error";
    }
    
    hand_pub.at("plan").at(req.ee_name).publish(msg); //publish on a topic when the trajectory is done
  
    busy.at(req.ee_name)=false;
    
    return;
}

void ikControl::execute_plan(dual_manipulation_shared::ik_service::Request req)
{
  ROS_INFO("IKControl::execute_plan: Executing plan for %s",req.ee_name.c_str());

  moveit::planning_interface::MoveItErrorCode error_code;
//   if (req.ee_name == "both_hands")
//   {
//     //split right and left arm plans
//     movePlans_.at("both_hands").trajectory_.joint_trajectory.header.stamp = ros::Time::now();
//     splitFullRobotPlan();
//     
//     traj_pub_.at("left_hand").publish(movePlans_.at("left_hand").trajectory_);
//     traj_pub_.at("right_hand").publish(movePlans_.at("right_hand").trajectory_);
//     
//     // TODO: manage errors here?
//     error_code.val = 1;
//   }
//   else
//   {
//     movePlans_.at(req.ee_name).trajectory_.joint_trajectory.header.stamp = ros::Time::now();
//     traj_pub_.at(req.ee_name).publish(movePlans_.at(req.ee_name).trajectory_);
//     
//     // TODO: manage errors here?
//     error_code.val = 1;
//   }
  
  // old execution method: does not allow for two trajectories at the same time
  error_code = moveGroups_.at(req.ee_name)->execute(movePlans_.at(req.ee_name));
  if(error_code.val == 1)
  {
    msg.data = "done";
  }
  else
  {
    msg.data = "error";
  }
  hand_pub.at("exec").at(req.ee_name).publish(msg); //publish on a topic when the trajectory is done

  busy.at(req.ee_name)=false;
  
  return;
}

bool ikControl::perform_ik(dual_manipulation_shared::ik_service::Request& req)
{
    if (!isInitialized_)
    {
      ROS_WARN("IKControl::perform_ik: robot model is not initialized - initialize it first!");
      return false;
    }

    if(!busy.count(req.ee_name))
    {
	ROS_ERROR("IKControl::perform_ik: Unknown end effector %s, returning",req.ee_name.c_str());
	return false;
    }

    if(req.ee_name == "both_hands" && ((req.command == "ik_check") || (req.command == "grasp") || (req.command == "ungrasp")))
    {
	ROS_ERROR("IKControl::perform_ik: Perform %s commands for each hand separately! Returning",req.command.c_str());
	return false;
    }

    // check for correctness when using one or both hands
    bool plan_bool;
    if (req.ee_name == "both_hands") 
      plan_bool = !(busy.at("left_hand") || busy.at("right_hand"));
    else
      plan_bool = !(busy.at(req.ee_name));
    
    if(plan_bool && !busy.at("both_hands"))
    {
	busy.at(req.ee_name)=true;
	if(req.command == "plan")
	{
	  std::thread* th = new std::thread(&ikControl::planning_thread,this, req);
	}
	else if(req.command == "ik_check")
	{
	  std::thread* th = new std::thread(&ikControl::ik_check_thread,this, req);
	}
	else if(req.command == "execute")
	{
	  std::thread* th = new std::thread(&ikControl::execute_plan,this, req);
	}
	else if(req.command == "home")
	{
	  std::thread* th = new std::thread(&ikControl::simple_homing,this, req.ee_name);
	}
	else if(req.command == "grasp")
	{
	  std::thread* th = new std::thread(&ikControl::grasp,this, req);
	}
	else if(req.command == "ungrasp")
	{
	  std::thread* th = new std::thread(&ikControl::ungrasp,this, req);
	}
	else
	{
	  ROS_WARN("IKControl::perform_ik: Unknown command: %s",req.command.c_str());
	  return false;
	}
    }
    else
    {
	ROS_WARN("IKControl::perform_ik: Already performing a %s ik_service",req.ee_name.c_str());
	return false;
    }

    return true;
}

ikControl::~ikControl()
{
    delete kinematics_plugin_.at("left_hand");
    delete kinematics_plugin_.at("right_hand");
    
    delete moveGroups_.at("left_hand");
    delete moveGroups_.at("right_hand");
    delete moveGroups_.at("both_hands");
}

bool ikControl::splitFullRobotPlan()
{
  int start_left,end_left,start_right,end_right;
  
  std::vector<std::string> active_joints_both = moveGroups_.at("both_hands")->getActiveJoints();
  std::vector<std::string> active_joints_left = moveGroups_.at("left_hand")->getActiveJoints();
  std::vector<std::string> active_joints_right = moveGroups_.at("right_hand")->getActiveJoints();
  
  std::vector <std::string >::iterator tmp;
  
  tmp = std::find(active_joints_both.begin(), active_joints_both.end(), active_joints_left.front());
  if (tmp != active_joints_both.end())
    start_left = tmp - active_joints_both.begin();
  else
  {
    ROS_ERROR("Left joint %s not found in full_robot group",active_joints_left.front().c_str());
    return false;
  }
  
  tmp = std::find(active_joints_both.begin(), active_joints_both.end(), active_joints_left.back());
  if (tmp != active_joints_both.end())
    end_left = tmp - active_joints_both.begin();
  else
  {
    ROS_ERROR("Left joint %s not found in full_robot group",active_joints_left.back().c_str());
    return false;
  }
  
  tmp = std::find(active_joints_both.begin(), active_joints_both.end(), active_joints_right.front());
  if (tmp != active_joints_both.end())
    start_right = tmp - active_joints_both.begin();
  else
  {
    ROS_ERROR("Right joint %s not found in full_robot group",active_joints_left.front().c_str());
    return false;
  }
  
  tmp = std::find(active_joints_both.begin(), active_joints_both.end(), active_joints_right.back());
  if (tmp != active_joints_both.end())
    end_right = tmp - active_joints_both.begin();
  else
  {
    ROS_ERROR("Right joint %s not found in full_robot group",active_joints_left.back().c_str());
    return false;
  }
  
  trajectory_msgs::JointTrajectoryPoint tmp_traj;
  
  // std::cout << "start_left | end_left | start_right | end_right = " << start_left << " | " << end_left << " | " << start_right << " | " << end_right << std::endl;
  
  for (auto item:movePlans_.at("both_hands").trajectory_.joint_trajectory.points)
  {
    // clear trajectory point
    tmp_traj.positions.clear();
    tmp_traj.velocities.clear();
    tmp_traj.accelerations.clear();
    tmp_traj.effort.clear();
    tmp_traj.time_from_start = item.time_from_start;
    
    // get left joint values
    if (!item.positions.empty())
      tmp_traj.positions.insert(tmp_traj.positions.end(),item.positions.begin()+start_left,item.positions.begin()+end_left+1);
    if (!item.velocities.empty())
      tmp_traj.velocities.insert(tmp_traj.velocities.end(),item.velocities.begin()+start_left,item.velocities.begin()+end_left+1);
    if (!item.accelerations.empty())
      tmp_traj.accelerations.insert(tmp_traj.accelerations.end(),item.accelerations.begin()+start_left,item.accelerations.begin()+end_left+1);
    if (!item.effort.empty())
      tmp_traj.effort.insert(tmp_traj.effort.end(),item.effort.begin()+start_left,item.effort.begin()+end_left+1);
    
    // std::cout << "left_hand trajectory point" << std::endl;
    // std::cout << tmp_traj << std::endl;
    
    // push them in the left hand trajectory
    movePlans_.at("left_hand").trajectory_.joint_trajectory.points.push_back(tmp_traj);
    
    // clear trajectory point
    tmp_traj.positions.clear();
    tmp_traj.velocities.clear();
    tmp_traj.accelerations.clear();
    tmp_traj.effort.clear();
    tmp_traj.time_from_start = item.time_from_start;
    
    // get right joint values
    if (!item.positions.empty())
      tmp_traj.positions.insert(tmp_traj.positions.end(),item.positions.begin()+start_right,item.positions.begin()+end_right+1);
    if (!item.velocities.empty())
      tmp_traj.velocities.insert(tmp_traj.velocities.end(),item.velocities.begin()+start_right,item.velocities.begin()+end_right+1);
    if (!item.accelerations.empty())
      tmp_traj.accelerations.insert(tmp_traj.accelerations.end(),item.accelerations.begin()+start_right,item.accelerations.begin()+end_right+1);
    if (!item.effort.empty())
      tmp_traj.effort.insert(tmp_traj.effort.end(),item.effort.begin()+start_right,item.effort.begin()+end_right+1);
    
    // std::cout << "right_hand trajectory point" << std::endl;
    // std::cout << tmp_traj << std::endl;
    
    // push them in the right hand trajectory
    movePlans_.at("right_hand").trajectory_.joint_trajectory.points.push_back(tmp_traj);    
  }
  
  // // fill in the remaining part of the plan
  
  // // these parts are useless
  // movePlans_.at("left_hand").start_state_ = movePlans_.at("both_hands").start_state_;
  // movePlans_.at("right_hand").start_state_ = movePlans_.at("both_hands").start_state_;
  // 
  // movePlans_.at("left_hand").planning_time_ = movePlans_.at("both_hands").planning_time_;
  // movePlans_.at("right_hand").planning_time_ = movePlans_.at("both_hands").planning_time_;
  
  movePlans_.at("left_hand").trajectory_.joint_trajectory.header = movePlans_.at("both_hands").trajectory_.joint_trajectory.header;
  movePlans_.at("right_hand").trajectory_.joint_trajectory.header = movePlans_.at("both_hands").trajectory_.joint_trajectory.header;

  movePlans_.at("left_hand").trajectory_.joint_trajectory.joint_names.clear();
  movePlans_.at("left_hand").trajectory_.joint_trajectory.joint_names.insert( movePlans_.at("left_hand").trajectory_.joint_trajectory.joint_names.end(), active_joints_left.begin(), active_joints_left.end() );
  movePlans_.at("right_hand").trajectory_.joint_trajectory.joint_names.clear();
  movePlans_.at("right_hand").trajectory_.joint_trajectory.joint_names.insert( movePlans_.at("right_hand").trajectory_.joint_trajectory.joint_names.end(), active_joints_right.begin(), active_joints_right.end() );
  
  return true;
}

bool ikControl::moveHand(std::string& hand, std::vector< double >& q, std::vector< double >& t)
{
  trajectory_msgs::JointTrajectory grasp_traj;
  
  grasp_traj.header.stamp = ros::Time::now();
  grasp_traj.joint_names.push_back(hand + "_synergy_joint");
  
  if (t.size() != q.size())
  {
    ROS_WARN("IKControl::moveHand: timing vector size non compatible with joint vector size, using a default timing of 1 second");
    t.clear();
    for (int i=0; i<q.size(); ++i)
      t.push_back(1.0/q.size()*i);
  }
  
  trajectory_msgs::JointTrajectoryPoint tmp_traj;
  tmp_traj.positions.reserve(1);
  
  for (int i=0; i<q.size(); ++i)
  {
    tmp_traj.positions.clear();
    tmp_traj.positions.push_back(q.at(i));
    tmp_traj.time_from_start = ros::Duration(t.at(i));
  
    grasp_traj.points.push_back(tmp_traj);
  }
  
  hand_synergy_pub_.at(hand).publish(grasp_traj);
  
  return true;
}

bool ikControl::moveHand(std::string& hand, trajectory_msgs::JointTrajectory& grasp_traj)
{
  hand_synergy_pub_.at(hand).publish(grasp_traj);
  
  return true;
}

void ikControl::simple_homing(std::string ee_name)
{
  ROS_INFO("IKControl::simple_homing: going back home...");

  moveGroups_.at(ee_name)->setNamedTarget( group_map_.at(ee_name) + "_home" );
  moveGroups_.at(ee_name)->setStartStateToCurrentState();
  
//   moveit::planning_interface::MoveItErrorCode error_code = moveGroups_.at(ee_name)->asyncMove();
  moveit::planning_interface::MoveItErrorCode error_code = moveGroups_.at(ee_name)->plan(movePlans_.at(ee_name));
  error_code = moveGroups_.at(ee_name)->asyncExecute(movePlans_.at(ee_name));
  
  busy.at(ee_name) = false;
  if(error_code.val == 1)
  {
    msg.data = "done";
  }
  else
  {
    msg.data = "error";
  }

  hand_pub.at("exec").at(ee_name).publish(msg);
  
  return;
}

void ikControl::grasp(dual_manipulation_shared::ik_service::Request req)
{
  ROS_INFO("IKControl::grasp: %s with %s",req.attObject.object.id.c_str(),req.ee_name.c_str());

  moveit::planning_interface::MoveItErrorCode error_code;
  
  //check whether the object was present, and in case remove it from the environment
  dual_manipulation_shared::scene_object_service::Request req_obj;
  req_obj.command = "attach";
  req_obj.object_id = req.attObject.object.id;
  req_obj.attObject = req.attObject;
  // insert the links which does not constitute a collision
  req_obj.attObject.touch_links.insert(req_obj.attObject.touch_links.begin(),allowed_collisions_.at(req.ee_name).begin(),allowed_collisions_.at(req.ee_name).end());
  attachObject(req_obj);
  
// // // // // // // // // // // // actual grasping // // // // // // // // // // // // 
  // compute waypoints
  // eef_step is set to a high value in order to only consider waypoints passed to the function (otherwise, intermediate waypoints are added)
  double eef_step = 1.0;
  // jump threshold set to 0 is unactive (it's just an a posteriori check anyway: if the distance between consecuteive waypoints is greater than jump_threshold*average distance, the trajectory is considered wrong and truncated right before that waypoint)
  double jump_threshold = 0.0;
  moveit_msgs::RobotTrajectory trajectory;
  bool avoid_collisions = true;
  moveGroups_.at(req.ee_name)->computeCartesianPath(req.ee_pose,eef_step,jump_threshold,trajectory,avoid_collisions,&error_code);

  // interpolate them
  robot_trajectory::RobotTrajectory robot_traj(moveGroups_.at(req.ee_name)->getCurrentState()->getRobotModel(),group_map_.at(req.ee_name));
  robot_traj.setRobotTrajectoryMsg(*(moveGroups_.at(req.ee_name)->getCurrentState()),trajectory);
  trajectory_processing::IterativeParabolicTimeParameterization iptp;
  iptp.computeTimeStamps(robot_traj);
  robot_traj.getRobotTrajectoryMsg(trajectory);
  
  // allign trajectories in time
  if (trajectory.joint_trajectory.points.size() == req.grasp_trajectory.points.size())
  {
    for (int i=0; i<trajectory.joint_trajectory.points.size(); ++i)
    {
      req.grasp_trajectory.points.at(i).time_from_start = trajectory.joint_trajectory.points.at(i).time_from_start;
    }
  }
  else
  {
    ROS_WARN_STREAM("IKControl::grasp: unequal length arm (" << trajectory.joint_trajectory.points.size() << ") and hand (" << req.grasp_trajectory.points.size() << ") trajectories: interpolating hand trajectory");
    ros::Duration delta_t = trajectory.joint_trajectory.points.back().time_from_start*(1.0/req.grasp_trajectory.points.size());
    for (int i=0; i<req.grasp_trajectory.points.size(); ++i)
    {
      req.grasp_trajectory.points.at(i).time_from_start = delta_t*(i+1);
    }
  }
  trajectory.joint_trajectory.header.stamp = ros::Time::now();
  req.grasp_trajectory.header.stamp = trajectory.joint_trajectory.header.stamp;
  
  movePlans_.at(req.ee_name).trajectory_ = trajectory;

  // execute both, after attaching the object to the end-effector
  // NOTE: here collision checking should probably be stopped
  error_code = moveGroups_.at(req.ee_name)->asyncExecute(movePlans_.at(req.ee_name));
  if (!moveHand(req.ee_name,req.grasp_trajectory) & error_code.val == 1)
  {
    error_code.val = -1000;
  }
// // // // // // // // // // // // actual grasping // // // // // // // // // // // // 
  
  if (error_code.val == -1000)
  {
    ROS_ERROR("IKControl::grasp: moveHand returned an error");
  }
  else if (error_code.val != 1)
  {
    ROS_ERROR("IKControl::grasp: computeCartesianPath returned the error ID %d",error_code.val);
    //TODO: handle errors in here
  }
  
  busy.at(req.ee_name) = false;
  if(error_code.val == 1)
  {
    msg.data = "done";
  }
  else
  {
    msg.data = "error";
  }

  hand_pub.at("grasp").at(req.ee_name).publish(msg);
  
  return;
}

void ikControl::ungrasp(dual_manipulation_shared::ik_service::Request req)
{
  ROS_INFO("IKControl::ungrasp: %s from %s",req.attObject.object.id.c_str(),req.ee_name.c_str());

  moveit::planning_interface::MoveItErrorCode error_code;
  error_code.val = 1;
  
  if((!grasped_objects_map_.count(req.attObject.object.id)) || (grasped_objects_map_.at(req.attObject.object.id).link_name.compare(ee_map_.at(req.ee_name))!=0))
  {
    ROS_WARN("IKControl::ungrasp: object with ID \"%s\" is not grasped by %s. Performing ungrasp action anyway",req.attObject.object.id.c_str(),req.ee_name.c_str());
  }
  else
  {
    dual_manipulation_shared::scene_object_service::Request req_scene;
    req_scene.command = "detach";
    req_scene.attObject = req.attObject;
  
    if (!addObject(req_scene))
    {
      error_code.val = 0;
    }
  }
  
  // open the hand
  std::vector <double > q = {0.0};
  std::vector <double > t = {0.5};
  if (!moveHand(req.ee_name,q,t))
    error_code.val = -1;
  
  busy.at(req.ee_name) = false;
  
  if(error_code.val == 1)
  {
    msg.data = "done";
  }
  else
  {
    msg.data = "error";
  }
  
  hand_pub.at("grasp").at(req.ee_name).publish(msg);
  
  return;
}
