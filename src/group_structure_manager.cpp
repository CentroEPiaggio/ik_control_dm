#include "dual_manipulation_ik_control/group_structure_manager.h"
#include "dual_manipulation_shared/parsing_utils.h"
#include <assert.h>
#include <iostream>
#include <string>
#include <algorithm>

#define RED_INTENSE "\033[0;91m"
#define NO_COLOR "\033[0m"
#define CLASS_NAMESPACE "GroupStructureManager::"

GroupStructureManager::GroupStructureManager(XmlRpc::XmlRpcValue& params)
{
    parseParameters(params);
}

void GroupStructureManager::parseParameters(XmlRpc::XmlRpcValue& params)
{
    assert(params.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    
    bool correct(true);
    
    parseSingleParameter(params,chain_names_list_,"chain_group_names",1);
    parseSingleParameter(params,tree_names_list_,"tree_group_names",1);
    
    // list of chains composing each tree
    if(params.hasMember("tree_composition"))
    {
        std::map<std::string,std::vector<std::string>> tc_tmp;
        for(auto tree:tree_names_list_)
        {
            parseSingleParameter(params["tree_composition"],tc_tmp[tree],tree);
            if(tc_tmp.at(tree).empty())
                tc_tmp.erase(tree);
            else
                for(auto chain:tc_tmp.at(tree))
                {
                    if(std::find(chain_names_list_.begin(),chain_names_list_.end(),chain) == chain_names_list_.end())
                    {
                        std::cout << RED_INTENSE << CLASS_NAMESPACE << __func__ << " : bad chain name \'" << chain << "\' specified in composition for tree '" << tree << "': check the yaml configuration." << NO_COLOR << std::endl;
                        correct = false;
                    }
                }
        }
        if(!tc_tmp.empty())
        {
            tree_composition_.swap(tc_tmp);
            tc_tmp.clear();
        }
    }
    std::vector<std::string> tree_names_tmp;
    tree_names_tmp.swap(tree_names_list_);
    for(auto tree:tree_names_tmp)
        if(!tree_composition_.count(tree) || tree_composition_.at(tree).size() == 0)
        {
            std::cout << RED_INTENSE << CLASS_NAMESPACE << __func__ << " : No composition is specified for tree '" << tree << "': check the yaml configuration." << NO_COLOR << std::endl;
            correct = false;
        }
        else
            tree_names_list_.push_back(tree);
    
    std::map<std::string,std::string> map_tmp,map_tmp_tree;
    parseSingleParameter(params,map_tmp,"group_map",chain_names_list_);
    parseSingleParameter(params,map_tmp_tree,"group_map",tree_names_list_);
    if(!map_tmp_tree.empty())
        for(auto tree:map_tmp_tree)
            map_tmp[tree.first] = tree.second;
    if(!map_tmp.empty())
    {
        group_map_.swap(map_tmp);
        map_tmp.clear();
    }
    if(group_map_.size() != chain_names_list_.size() + tree_names_list_.size())
    {
        std::cout << RED_INTENSE << CLASS_NAMESPACE << __func__ << " : group_map parameter has size " << group_map_.size() << ", but it should be equal to the sum of chains and trees (" << chain_names_list_.size() << " + " << tree_names_list_.size() << "): check the yaml configuration." << NO_COLOR << std::endl;
        correct = false;
    }
    
    assert(correct);
}

bool GroupStructureManager::getGroupInSRDF(const std::string& ik_control_group, std::string& group_name) const
{
    if(group_map_.count(ik_control_group))
    {
        group_name = group_map_.at(ik_control_group);
        return true;
    }
    
    return false;
}

const std::map< std::string, std::string >& GroupStructureManager::get_group_map() const
{
    return group_map_;
}

const std::vector< std::string >& GroupStructureManager::get_chains() const
{
    return chain_names_list_;
}

bool GroupStructureManager::is_chain(const std::string& group) const
{
    return std::find(chain_names_list_.begin(),chain_names_list_.end(),group) != chain_names_list_.end();
}

bool GroupStructureManager::is_tree(const std::string& group) const
{
    return std::find(tree_names_list_.begin(),tree_names_list_.end(),group) != tree_names_list_.end();
}

const std::vector< std::string >& GroupStructureManager::get_tree_composition(const std::string& group) const
{
    if(!is_tree(group))
        return empty_vector;
    
    return tree_composition_.at(group);
}

std::vector< std::string > GroupStructureManager::get_trees_with_chain(const std::string& group) const
{
    std::vector<std::string> res;
    if(!is_chain(group)) return res;
    
    for(auto tree:tree_names_list_)
    {
        if(std::find(tree_composition_.at(tree).begin(),tree_composition_.at(tree).end(),group) != tree_composition_.at(tree).end())
            res.push_back(tree);
    }
    
    return res;
}
