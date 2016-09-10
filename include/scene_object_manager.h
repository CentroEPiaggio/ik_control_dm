#ifndef SCENE_OBJECT_MANAGER_H
#define SCENE_OBJECT_MANAGER_H

#include <ros/ros.h>
#include <dual_manipulation_shared/scene_object_service.h>
#include <dual_manipulation_shared/databasemapper.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <mutex>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

namespace dual_manipulation
{
namespace ik_control
{

static const std::string ADD_OBJECT("add");
static const std::string REMOVE_OBJECT("remove");
static const std::string ATTACH_OBJECT("attach");
static const std::string DETACH_OBJECT("detach");
static const std::string REMOVE_ALL_OBJECTS("remove_all");

/**
  * @brief This is a class that is used from the ros_server to manage objects in the scene used for planning.
  * 
  */
class SceneObjectManager
{
public:
  
    SceneObjectManager();
    ~SceneObjectManager() {}

    /**
     * @brief interface function to manage objects
     * 
     * @param req
     *   the req from the @e scene_object_service
     * @return bool
     */
    bool manage_object(dual_manipulation_shared::scene_object_service::Request &req);
    
private:
    
    ros::NodeHandle node;
    
    std::mutex map_mutex_;
    std::map<std::string,moveit_msgs::AttachedCollisionObject> grasped_objects_map_;
    std::map<std::string,moveit_msgs::AttachedCollisionObject> world_objects_map_;
    ros::Publisher collision_object_publisher_,attached_collision_object_publisher_;
    planning_scene_monitor::PlanningSceneMonitorPtr scene_monitor_;
    
    std::unique_ptr<databaseMapper> db_mapper_;
    
    /**
     * @brief utility function to load a mesh and attach it to a collision object
     * 
     * @param req
     *   the same req from @e scene_object_service
     *   contains the attached collision object to which the mesh has to be attached
     *   this is obtained from the loaded database, using the identifier @p req.object_db_id
     * 
     * @return void
     */
    void loadAndAttachMesh(dual_manipulation_shared::scene_object_service::Request& req);
    
    /**
     * @brief insert a new object in the planning scene
     * 
     * @param req
     *   the same req from @e scene_object_service
     * @return bool
     */
    bool addObject(dual_manipulation_shared::scene_object_service::Request req);
    
    /**
     * @brief remove an object from the planning scene
     * 
     * @param object_id
     *   the id of the object to be removed from the scene
     * @return bool
     */
    bool removeObject(std::string &object_id);
    
    /**
     * @brief an object present in the planning scene becomes attached to a robot link
     * 
     * @param req
     *   the the same req from @e scene_object_service
     * @return bool
     */
    bool attachObject(dual_manipulation_shared::scene_object_service::Request& req);
    
    /**
     * @brief remove all objects stored in internal structures from the planning scene
     * 
     * @return true on success
     */
    bool removeAllObjects();
    
    /**
     * @brief get a full planning scene and initialize internal variables as appropriate
     */
    void initializeSceneObjectsAndMonitor();
    
    /**
     * @brief get a full planning scene and initialize internal variables as appropriate
     */
    void initializeSceneMonitor(const moveit_msgs::PlanningScene& scene);
};

}
}

#endif // SCENE_OBJECT_MANAGER_H
