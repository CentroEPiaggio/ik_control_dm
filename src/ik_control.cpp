#include "ik_control.h"
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <thread>

using namespace dual_manipulation::ik_control;

ikControl::ikControl()
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
    
    robot_state_publisher_ = node.advertise<moveit_msgs::DisplayRobotState>( "/ik_control/robot_state", 1 );
    
    kinematics_plugin_["left_hand"] = new kdl_kinematics_plugin::KDLKinematicsPlugin();
    kinematics_plugin_["right_hand"] = new kdl_kinematics_plugin::KDLKinematicsPlugin();
    
    group_map_["left_hand"] = "left_hand_arm";
    group_map_["right_hand"] = "right_hand_arm";
    group_map_["both_hands"] = "full_robot";
    
    moveGroups_["left_hand"] = new move_group_interface::MoveGroup(group_map_.at("left_hand"));
    moveGroups_["right_hand"] = new move_group_interface::MoveGroup(group_map_.at("right_hand"));
    moveGroups_["both_hands"] = new move_group_interface::MoveGroup(group_map_.at("both_hands"));
    
    // unconmment to set a different tolerance (to 0.005 m / 0.005 rad = 0.5 degree in this case)
    for(auto item:moveGroups_)
    {
      item.second->setGoalPositionTolerance(0.005);
      item.second->setGoalOrientationTolerance(0.005);
    }

    movePlans_["left_hand"];
    movePlans_["right_hand"];
    movePlans_["both_hands"];
    
    // NOTE: attempted value of search_discretization: it's not clear what it is used for
    kinematics_plugin_.at("left_hand")->initialize("robot_description","left_hand_arm","world","left_hand_palm_link",0.005);
    kinematics_plugin_.at("right_hand")->initialize("robot_description","right_hand_arm","world","right_hand_palm_link",0.005);
    
    isInitialized_ = true;
    
    // planning scene differential publisher and differential PlanningScene setup
    planning_scene_diff_publisher_ = node.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    planning_scene_.is_diff = true;
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
    return attachObject(req.attObject);
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

bool ikControl::addObject(dual_manipulation_shared::scene_object_service::Request req)
{
  moveit_msgs::AttachedCollisionObject attObject = req.attObject;
  
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
      ROS_WARN_STREAM("The object to detach (" << req.attObject.object.id << ") was not present in the scene");
      return false;
    }
  }
  
  planning_scene_.robot_state.attached_collision_objects.clear();
  planning_scene_.world.collision_objects.clear();
  planning_scene_.world.collision_objects.push_back(attObject.object);
  planning_scene_diff_publisher_.publish(planning_scene_);
  planning_scene_.world.collision_objects.clear();
  
  // store information about this object in a class map
  world_objects_map_[attObject.object.id] = attObject;
  
  return true;
}

bool ikControl::removeObject(std::string& object_id)
{
  // remove associated information about this object
  if (!(world_objects_map_.count(object_id) || grasped_objects_map_.count(object_id) ))
  {
    ROS_WARN_STREAM("The object " << object_id << " was not in the environment!");
    return false;
  }
  else
  {
    std::string where;
    
    planning_scene_.robot_state.attached_collision_objects.clear();
    planning_scene_.world.collision_objects.clear();

    if (world_objects_map_.count(object_id))
    {
      where = world_objects_map_.at(object_id).object.header.frame_id;
      /* Define the message and publish the operation*/
      moveit_msgs::CollisionObject remove_object;
      remove_object.id = object_id;
      remove_object.header.frame_id = world_objects_map_.at(object_id).object.header.frame_id;
      remove_object.operation = remove_object.REMOVE;

      planning_scene_.world.collision_objects.push_back(remove_object);
      // erase the object from the map
      world_objects_map_.erase( world_objects_map_.find(object_id) );
    }
    else if (grasped_objects_map_.count(object_id))
    {
      where = grasped_objects_map_.at(object_id).link_name;
      // TODO: check whether or not this puts back the object in the world
      moveit_msgs::AttachedCollisionObject detach_object;
      detach_object.object.id = object_id;
      detach_object.link_name = grasped_objects_map_.at(object_id).link_name;
      detach_object.object.operation = detach_object.object.REMOVE;
      planning_scene_.robot_state.attached_collision_objects.push_back(detach_object);
      // erase the object from the map
      grasped_objects_map_.erase( grasped_objects_map_.find(object_id) );
    }
    
    planning_scene_diff_publisher_.publish(planning_scene_);
    planning_scene_.robot_state.attached_collision_objects.clear();
    planning_scene_.world.collision_objects.clear();
    ROS_INFO_STREAM("Object " << object_id << " removed from " << where);
  }
  
  return true;
}

bool ikControl::attachObject(moveit_msgs::AttachedCollisionObject& attObject)
{
  // remove associated information about this object
  if (!world_objects_map_.count(attObject.object.id))
  {
    ROS_WARN_STREAM("The object " << attObject.object.id << " is not in the environment!");
    return false;
  }
  else
  {
    // remove the object from where it is currently
    removeObject(attObject.object.id);
    
    // attach it to the robot
    planning_scene_.robot_state.attached_collision_objects.push_back(attObject);
    planning_scene_diff_publisher_.publish(planning_scene_);
    
    planning_scene_.robot_state.attached_collision_objects.clear();
    
    // store information about this object in a class map
    grasped_objects_map_[attObject.object.id] = attObject;
  }
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
  
  ROS_INFO_STREAM("IKControl::ik_check_thread: error_code.val = " << error_code.val << std::endl);
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
    localMoveGroup->plan(*movePlan);
    
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

    msg.data = "done";
    hand_pub.at("plan").at(req.ee_name).publish(msg); //publish on a topic when the trajectory is done
  
    busy.at(req.ee_name)=false;
    
    return;
}

void ikControl::execute_plan(dual_manipulation_shared::ik_service::Request req)
{
  ROS_INFO("IKControl::execute_plan: Executing plan for %s",req.ee_name.c_str());

  moveit::planning_interface::MoveItErrorCode error_code;
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

    if(req.ee_name == "both_hands" && req.command == "ik_check")
    {
	ROS_ERROR("IKControl::perform_ik: Perform IK for each hand separately! Returning");
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
	  execute_plan(req);
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
