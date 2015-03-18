#ifndef SCENE_OBJECT_MANAGER_H
#define SCENE_OBJECT_MANAGER_H

#include <ros/ros.h>
#include <dual_manipulation_shared/scene_object_service.h>
#include <dual_manipulation_shared/databasemapper.h>
#include <moveit_msgs/AttachedCollisionObject.h>

namespace dual_manipulation
{
namespace ik_control
{
  
/**
  * @brief This is a class that is used from the ros_server to manage objects in the scene used for planning.
  * 
  */
class sceneObjectManager
{
public:
  
    sceneObjectManager();
    ~sceneObjectManager();

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
    
    std::map<std::string,moveit_msgs::AttachedCollisionObject> grasped_objects_map_;
    std::map<std::string,moveit_msgs::AttachedCollisionObject> world_objects_map_;
    ros::Publisher collision_object_publisher_,attached_collision_object_publisher_;
    
    databaseMapper* db_mapper_;
    
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

};

}
}

#endif // SCENE_OBJECT_MANAGER_H