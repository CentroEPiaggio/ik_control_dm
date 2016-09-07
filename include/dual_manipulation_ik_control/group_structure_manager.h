#ifndef GROUP_STRUCTURE_MANAGER_H
#define GROUP_STRUCTURE_MANAGER_H

#include <map>
#include <XmlRpcValue.h>

/**
 * @brief A class to manage the group structure used in dual_manipulation_ik_control
 * 
 * This class allows the conversion between names used in ik_control and group names defined in the robot SRDF file using the parameters loaded on the server. It also provides some useful functionalities to find the right group to use in some circumstances.
 */
class GroupStructureManager
{
public:
    GroupStructureManager(XmlRpc::XmlRpcValue& params);
    ~GroupStructureManager() {}
    
    /**
     * @brief Return the SRDF group name associated to an ik_control group name
     * 
     * @return false if the group doesn't exist, true otherwise
     */
    bool getGroupInSRDF(const std::string& ik_control_group, std::string& group_name) const;
    
    // TODO: is_chain() + is_tree() + find_chains_in_tree() + find_trees_containig_chain()
    
private:
    std::map<std::string,std::string> group_map_;
    std::vector<std::string> chain_names_list_;
    std::vector<std::string> tree_names_list_;
    std::map<std::string,std::vector<std::string>> tree_composition_;
    
private:
    /**
     * @brief utility function to parse parameters and obtain the needed structure
     * 
     * @param params parameters got from the parameter server
     * 
     * @return void
     */
    void parseParameters(XmlRpc::XmlRpcValue& params);
};

#endif // GROUP_STRUCTURE_MANAGER_H
