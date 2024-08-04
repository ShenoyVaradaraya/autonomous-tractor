/**
 * @file subregion.cpp
 * @author Kumar Rahul Dinkar
 * @brief 1. Subregion class represents the input farmland region 
 *           and stores the corresponding planned straight paths. 
 * 
 *        2. This class was planned to represent convex regions in the input
 *           concave farmland region.
 *         
 * @version 0.1
 * @date 2022-1-10
 * @copyright Copyright (c) 2022 FLUX AUTO
 */

#include "line.h"
#include "subregion.h"
#include <iostream>

/**
 * @brief Construct a new subregion::subregion object
 * 
 * @param id subregion unique identifier
 */

subregion::subregion(const int id):id_(id){}

/**
 * @brief inserts a line in the path object of the subregion :: id
 * @param id 
 * @param tmp new planned line lying inside the subregion
 */
void subregion::insertLines(const int id, const line& tmp)
{
    if(id_ == id)
        path_.push_back(tmp);
        
    else{}
        //create exception
}

/**
 * @brief counts the number of unconnected straight lines in the path object
 * 
 * @return int size 
 */
int subregion::lineCount()
{
    return path_.size();
}

/**
 * @brief get the subregion id_
 * 
 * @return int 
 */
int subregion::checkID()
{
    return id_;
}

/**
 * @brief uses print object of the line class to display current paralle lines
 *        constituting the path 
 */
void subregion::printLines()
{
    std::cout<<"ID of the subregion"<<id_<<std::endl;
    for(auto& iter: path_)
    {
        iter.printXY();
    }
}

/**
 * @brief output is the path object
 * 
 * @return std::vector<line> 
 */
std::vector<line> subregion::getIterRegion()
{
    return path_;
}

/**
 * @brief can be used to assign a path to the subregion
 * 
 * @param path 
 */
void subregion::setIterRegion(std::vector<line>& path)
{
    path_.clear();
    path_ = path;
}