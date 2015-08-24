#include "scene_object_manager.h"
#include <dual_manipulation_shared/parsing_utils.h>

#include <shape_msgs/Mesh.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/mesh_operations.h>
#include <moveit_msgs/GetPlanningScene.h>

#define CLASS_NAMESPACE "sceneObjectManager::"
#define DEBUG 1

using namespace dual_manipulation::ik_control;

sceneObjectManager::sceneObjectManager()
{
    db_mapper_ = new databaseMapper();
    
    // publishers for objects in the scene
    attached_collision_object_publisher_ = node.advertise<moveit_msgs::AttachedCollisionObject>("/attached_collision_object",1);
    collision_object_publisher_ = node.advertise<moveit_msgs::CollisionObject>("/collision_object",1);
    
    // check if the object DB is loaded correctly
    std::cout << "Object DB:" << std::endl;
    for(auto item:db_mapper_->Objects)
      std::cout << " - " << item.first << ": " << std::get<0>(item.second) << " + " << std::get<1>(item.second) << std::endl;
    
    initializeSceneObjects();
}

sceneObjectManager::~sceneObjectManager()
{
    delete db_mapper_;
}

void sceneObjectManager::initializeSceneObjects()
{
    // for the first time, update the planning scene in full
    ros::ServiceClient scene_client = node.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");
    moveit_msgs::GetPlanningScene srv;
    uint32_t objects = moveit_msgs::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS | moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_NAMES | moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY;
    srv.request.components.components = objects;
    if(!scene_client.call(srv))
      ROS_WARN_STREAM(CLASS_NAMESPACE << __func__ << " : unable to call /get_planning_scene service - starting with an empty planning scene...");
    else
    {
      for(auto attObject:srv.response.scene.robot_state.attached_collision_objects)
      {
	grasped_objects_map_[attObject.object.id] = attObject;
	#if DEBUG
	ROS_INFO_STREAM(CLASS_NAMESPACE << __func__ << " : added attached collision object " << attObject.object.id);
	#endif
      }
      for(auto object:srv.response.scene.world.collision_objects)
      {
	moveit_msgs::AttachedCollisionObject attObject;
	attObject.object = object;
	world_objects_map_[object.id] = attObject;
	#if DEBUG
	ROS_INFO_STREAM(CLASS_NAMESPACE << __func__ << " : added collision object " << attObject.object.id);
	#endif
      }
      ROS_DEBUG_STREAM(CLASS_NAMESPACE << __func__ << " : /get_planning_scene service returned \n" << srv.response.scene);
    }
}

bool sceneObjectManager::manage_object(dual_manipulation_shared::scene_object_service::Request& req)
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
  else if (req.command == "remove_all")
  {
    return removeAllObjects();
  }
  else
  {
    ROS_ERROR("sceneObjectManager::manage_object: Unknown command %s, returning",req.command.c_str());
    return false;
  }
}

void sceneObjectManager::loadAndAttachMesh(dual_manipulation_shared::scene_object_service::Request& req)
{
  // load the appropriate mesh and add it to the CollisionObject
  moveit_msgs::AttachedCollisionObject& attObject = req.attObject;
  
  // NOTE: the mesh should be in ASCII format, in meters, and the frame of reference should be coherent with the external information we get (obviously...)
  shapes::Mesh* m;
  shape_msgs::Mesh co_mesh;
  shapes::ShapeMsg co_mesh_msg;
  
  m = shapes::createMeshFromResource("package://asus_scanner_models/" + std::get<1>(db_mapper_->Objects.at( (int)req.object_db_id )));
  m->scale(1); // change this to 0.001 if expressed in mm; this does not change the frame, thus if it's not baricentric the object will be moved around
  shapes::constructMsgFromShape(m,co_mesh_msg);
  co_mesh = boost::get<shape_msgs::Mesh>(co_mesh_msg);
  
  req.attObject.object.meshes.clear();
  req.attObject.object.meshes.push_back(co_mesh);
}

bool sceneObjectManager::addObject(dual_manipulation_shared::scene_object_service::Request req)
{
  req.attObject.object.operation = req.attObject.object.ADD;

  moveit_msgs::AttachedCollisionObject& attObject = req.attObject;
  
  loadAndAttachMesh(req);
  
  ROS_INFO_STREAM("sceneObjectManager::addObject : Putting the object " << attObject.object.id << " into the environment (" << req.attObject.object.header.frame_id << ")...");

  if ((req.command == "add") && (grasped_objects_map_.count(attObject.object.id)))
  {
    ROS_WARN_STREAM("sceneObjectManager::addObject : The object was already attached to " << grasped_objects_map_.at(attObject.object.id).link_name << ": moving it to the environment");
    removeObject(attObject.object.id);
  }
  else if (req.command == "detach")
  {
// // // //     return true;
    
    if (grasped_objects_map_.count(req.object_id))
    {
      attObject = grasped_objects_map_.at(req.object_id);
      removeObject(attObject.object.id);
    }
    else
    {
      ROS_WARN_STREAM("sceneObjectManager::addObject : The object to detach (" << req.attObject.object.id << ") was not attached to any end-effector: did you grasp it before?");
      return false;
    }
  }
  
  collision_object_publisher_.publish(attObject.object);
  
  // store information about this object in a class map
  world_objects_map_[attObject.object.id] = attObject;
  
  return true;
}

bool sceneObjectManager::removeObject(std::string& object_id)
{
  // remove associated information about this object
  if (!(world_objects_map_.count(object_id) || grasped_objects_map_.count(object_id) ))
  {
    ROS_WARN_STREAM("sceneObjectManager::removeObject : The object " << object_id << " was not in the scene!");
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
      
      // the object is put in the world by default
      world_objects_map_[object_id] = grasped_objects_map_.at(object_id);
      // erase the object from the map
      grasped_objects_map_.erase( grasped_objects_map_.find(object_id) );
    }
    
    ROS_INFO_STREAM("sceneObjectManager::removeObject : Object " << object_id << " removed from " << where);
  }
  
  return true;
}

bool sceneObjectManager::removeAllObjects()
{
  std::vector<std::string> objects;
  for(auto object:world_objects_map_)
    objects.push_back(object.first);
  for(auto object:grasped_objects_map_)
    objects.push_back(object.first);
  
  for(auto object:objects)
    removeObject(object);
  
  return true;
}

bool sceneObjectManager::attachObject(dual_manipulation_shared::scene_object_service::Request& req)
{
// // //   removeObject(req.object_id);
// // //   return true;
  
  req.attObject.object.operation = req.attObject.object.ADD;
  
  moveit_msgs::AttachedCollisionObject& attObject = req.attObject;
  
  loadAndAttachMesh(req);

  ROS_INFO("sceneObjectManager::attachObject : attaching the object %s to %s",attObject.object.id.c_str(),attObject.link_name.c_str());
  // remove associated information about this object
  if ((!world_objects_map_.count(attObject.object.id)) && (!grasped_objects_map_.count(attObject.object.id)))
  {
    ROS_WARN_STREAM("sceneObjectManager::attachObject : The object " << attObject.object.id << " is not in the scene! Attaching it to " << attObject.link_name << " anyway...");
  }
  else
  {
    // NOTE: to remove the object from where it is currently, if it's grasped, the procedure needs to be called twice!
    if(grasped_objects_map_.count(attObject.object.id))
    {
        removeObject(attObject.object.id);
        usleep(3000);
    }
    removeObject(attObject.object.id);
    usleep(3000);
  }
  
  // attach it to the robot
  attached_collision_object_publisher_.publish(attObject);
  
  // store information about this object in a class map
  grasped_objects_map_[attObject.object.id] = attObject;
  
  return true;
}
