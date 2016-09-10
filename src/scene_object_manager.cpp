#include "scene_object_manager.h"
#include <dual_manipulation_shared/parsing_utils.h>

#include <shape_msgs/Mesh.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/mesh_operations.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit/move_group/capability_names.h>

#define CLASS_NAMESPACE "SceneObjectManager::"
#define CLASS_LOGNAME "SceneObjectManager"
#define DEBUG 0

using namespace dual_manipulation::ik_control;

SceneObjectManager::SceneObjectManager()
{
    db_mapper_.reset(new databaseMapper());
    
    // publishers for objects in the scene
    attached_collision_object_publisher_ = node.advertise<moveit_msgs::AttachedCollisionObject>("attached_collision_object",1);
    collision_object_publisher_ = node.advertise<moveit_msgs::CollisionObject>("collision_object",1);
    
    // check if the object DB is loaded correctly
    std::cout << "Object DB:" << std::endl;
    for(auto item:db_mapper_->Objects)
        std::cout << " - " << item.first << ": " << std::get<0>(item.second) << " + " << std::get<1>(item.second) << std::endl;
    
    initializeSceneObjectsAndMonitor();
}

void SceneObjectManager::initializeSceneObjectsAndMonitor()
{
    // for the first time, update the planning scene in full
    ros::ServiceClient scene_client = node.serviceClient<moveit_msgs::GetPlanningScene>(move_group::GET_PLANNING_SCENE_SERVICE_NAME);
    moveit_msgs::GetPlanningScene srv;

    // I want to update the full scene: is there a better way than this?
    uint32_t full_scene = moveit_msgs::PlanningSceneComponents::SCENE_SETTINGS | moveit_msgs::PlanningSceneComponents::ROBOT_STATE | moveit_msgs::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS | moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_NAMES | moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY | moveit_msgs::PlanningSceneComponents::OCTOMAP | moveit_msgs::PlanningSceneComponents::TRANSFORMS | moveit_msgs::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX | moveit_msgs::PlanningSceneComponents::LINK_PADDING_AND_SCALING | moveit_msgs::PlanningSceneComponents::OBJECT_COLORS;
    
    srv.request.components.components = full_scene;
    if(!scene_client.call(srv))
        ROS_WARN_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : unable to call " << move_group::GET_PLANNING_SCENE_SERVICE_NAME << " - starting with an empty planning scene...");
    else
    {
        for(auto attObject:srv.response.scene.robot_state.attached_collision_objects)
        {
            grasped_objects_map_[attObject.object.id] = attObject;
            ROS_DEBUG_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : added attached collision object " << attObject.object.id);
        }
        for(auto object:srv.response.scene.world.collision_objects)
        {
            moveit_msgs::AttachedCollisionObject attObject;
            attObject.object = object;
            world_objects_map_[object.id] = attObject;
            ROS_DEBUG_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : added collision object " << attObject.object.id);
        }
    }
    
    initializeSceneMonitor(srv.response.scene);
}

void SceneObjectManager::initializeSceneMonitor(const moveit_msgs::PlanningScene& scene)
{
    // initialize planning scene monitor
    const boost::shared_ptr<tf::Transformer> tft(new tf::Transformer());
    scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor("/robot_description",tft));
    
    ros::NodeHandle nh("~");
    nh.setParam("octomap_frame","camera_rgb_optical_frame");
    nh.setParam("octomap_resolution",0.02);
    
    // start monitoring stuff - default behavior monitors everything on global topics
    bool monitor_octomap(true);
    scene_monitor_->startWorldGeometryMonitor(planning_scene_monitor::PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC, planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC, monitor_octomap);
    scene_monitor_->startStateMonitor(planning_scene_monitor::PlanningSceneMonitor::DEFAULT_JOINT_STATES_TOPIC, planning_scene_monitor::PlanningSceneMonitor::DEFAULT_ATTACHED_COLLISION_OBJECT_TOPIC);
    scene_monitor_->startSceneMonitor(planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_TOPIC);
    
    // start publishing the full scene
    planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType su_type = planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType::UPDATE_SCENE;
    scene_monitor_->startPublishingPlanningScene(su_type, planning_scene_monitor::PlanningSceneMonitor::MONITORED_PLANNING_SCENE_TOPIC);
    scene_monitor_->setPlanningScenePublishingFrequency(10);
    
    planning_scene_monitor::LockedPlanningSceneRW ls(scene_monitor_);
    ls->usePlanningSceneMsg(scene);
}

bool SceneObjectManager::manage_object(dual_manipulation_shared::scene_object_service::Request& req)
{
    if (req.command == dual_manipulation::ik_control::ADD_OBJECT)
    {
        return addObject(req);
    }
    else if (req.command == dual_manipulation::ik_control::REMOVE_OBJECT)
    {
        return removeObject(req.object_id);
    }
    else if (req.command == dual_manipulation::ik_control::ATTACH_OBJECT)
    {
        return attachObject(req);
    }
    else if (req.command == dual_manipulation::ik_control::DETACH_OBJECT)
    {
        return addObject(req);
    }
    else if (req.command == dual_manipulation::ik_control::REMOVE_ALL_OBJECTS)
    {
        return removeAllObjects();
    }
    else
    {
        ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : Unknown command " << req.command << ", returning");
        return false;
    }
}

void SceneObjectManager::loadAndAttachMesh(dual_manipulation_shared::scene_object_service::Request& req)
{
    // load the appropriate mesh and add it to the CollisionObject
    moveit_msgs::AttachedCollisionObject& attObject = req.attObject;
    
    // NOTE: the mesh should be in ASCII format, in meters, and the frame of reference should be coherent with the external information we get (obviously...)
    std::unique_ptr<shapes::Mesh> m;
    shape_msgs::Mesh co_mesh;
    shapes::ShapeMsg co_mesh_msg;
    
    m.reset(shapes::createMeshFromResource(std::get<1>(db_mapper_->Objects.at( (int)req.object_db_id ))));
    m->scale(1); // change this to 0.001 if expressed in mm; this does not change the frame, thus if it's not baricentric the object will be moved around
    shapes::constructMsgFromShape(m.get(),co_mesh_msg);
    co_mesh = boost::get<shape_msgs::Mesh>(co_mesh_msg);
    
    req.attObject.object.meshes.clear();
    req.attObject.object.meshes.push_back(co_mesh);
}

bool SceneObjectManager::addObject(dual_manipulation_shared::scene_object_service::Request req)
{
    req.attObject.object.operation = req.attObject.object.ADD;
    
    moveit_msgs::AttachedCollisionObject& attObject = req.attObject;
    
    loadAndAttachMesh(req);
    
    ROS_INFO_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : Putting the object " << attObject.object.id << " into the environment (" << req.attObject.object.header.frame_id << ")...");
    
    if ((req.command == dual_manipulation::ik_control::ADD_OBJECT) && (grasped_objects_map_.count(attObject.object.id)))
    {
        ROS_WARN_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : The object was already attached to " << grasped_objects_map_.at(attObject.object.id).link_name << ": moving it to the environment");
        removeObject(attObject.object.id);
    }
    else if (req.command == dual_manipulation::ik_control::DETACH_OBJECT)
    {
        if (grasped_objects_map_.count(req.object_id))
        {
            attObject = grasped_objects_map_.at(req.object_id);
            removeObject(attObject.object.id);
        }
        else
        {
            ROS_WARN_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : The object to detach (" << req.attObject.object.id << ") was not attached to any end-effector: did you grasp it before?");
            return false;
        }
    }
    
    collision_object_publisher_.publish(attObject.object);
    
    // store information about this object in a class map
    world_objects_map_[attObject.object.id] = attObject;
    
    return true;
}

bool SceneObjectManager::removeObject(std::string& object_id)
{
    // remove associated information about this object
    if (!(world_objects_map_.count(object_id) || grasped_objects_map_.count(object_id) ))
    {
        ROS_WARN_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : The object " << object_id << " was not in the scene!");
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
        
        ROS_INFO_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : Object " << object_id << " removed from " << where);
    }
    
    return true;
}

bool SceneObjectManager::removeAllObjects()
{
    // NOTE: call the remove procedure for each object, twice if it is a grasped object!
    
    std::vector<std::string> objects;
    for(auto object:grasped_objects_map_)
        objects.push_back(object.first);
    for(auto object:objects)
        removeObject(object);
    
    for(auto object:world_objects_map_)
        objects.push_back(object.first);
    for(auto object:objects)
        removeObject(object);
    
    return true;
}

bool SceneObjectManager::attachObject(dual_manipulation_shared::scene_object_service::Request& req)
{
    req.attObject.object.operation = req.attObject.object.ADD;
    
    moveit_msgs::AttachedCollisionObject& attObject = req.attObject;
    
    loadAndAttachMesh(req);
    
    ROS_INFO_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : attaching the object " << attObject.object.id << " to " << attObject.link_name);
    // remove associated information about this object
    if ((!world_objects_map_.count(attObject.object.id)) && (!grasped_objects_map_.count(attObject.object.id)))
    {
        ROS_WARN_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : The object " << attObject.object.id << " is not in the scene! Attaching it to " << attObject.link_name << " anyway...");
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
