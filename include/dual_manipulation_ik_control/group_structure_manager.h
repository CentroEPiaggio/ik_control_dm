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
    
    /**
     * @brief Return a map of ik_control > SRDF group names
     */
    const std::map<std::string,std::string>& get_group_map() const;
    
    /**
     * @brief Return a vector containing known end-effectors (chain-type groups)
     */
    const std::vector<std::string>& get_chains() const;
    
    /**
     * @brief True if the ik_control group passed as input is a chain
     */
    bool is_chain(const std::string& group) const;
    
    /**
     * @brief True if the ik_control group passed as input is a tree
     */
    bool is_tree(const std::string& group) const;
    
    /**
     * @brief Return a vector containing all chains in a tree (empty if the tree is not present)
     */
    const std::vector<std::string>& get_tree_composition(const std::string& group) const;
    
    /**
     * @brief Return a vector containing all trees which contain the chain
     */
    std::vector<std::string> get_trees_with_chain(const std::string& group) const;
    
private:
    std::map<std::string,std::string> group_map_;
    std::vector<std::string> chain_names_list_;
    std::vector<std::string> tree_names_list_;
    std::map<std::string,std::vector<std::string>> tree_composition_;
    
    std::vector<std::string> empty_vector;
    
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
