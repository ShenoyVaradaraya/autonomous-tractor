/**
 * @file line.cpp
 * @author Kumar Rahul Dinkar
 * @brief Line class is used to represent a line passing through the input map
 * @version 0.1
 * @date 2022-01-10
 * 
 * @copyright Copyright (c) 2022 FLUX AUTO
 * 
 */
#include "line.h"
#include <math.h>
#include <iostream>

/**
 * @brief Construct a new line::line object
 * 
 * @param id stores the direction perpendicular to which this line is drawn.
 */
line::line(const double id):optimalDirection_(id), checked_once_(false), bifurcation_point_(false){}

/**
 * @brief Calculates the length of the line 
 * 
 * @return double 
 */
double line::getLength()
{
    double length;
    length = std::pow(std::pow(init_.first - end_.first, 2)+std::pow(init_.second-end_.second, 2),0.5);
    return length;
}

/**
 * @brief Calculates the angle of inclination of the line.
 * 
 * @return double 
 */
double line::getTheta()
{
    double theta;
    theta = atan2f(init_.second - end_.second , init_.first - end_.first);
    return theta;
}
 
 /**
  * @brief sets the init_ object with the input point having lesser y-ccordinate value 
  *        and the end_ point with the another point
  * 
  * @param pt1 one of the terminal points of the line
  * @param pt2 second terminal point of the line
  */
void line::setInput(const std::pair<double, double>& pt1, const std::pair<double, double>& pt2)
{
    if(pt1.second <= pt2.second)
    {    
        init_ = pt1;
        end_ = pt2;
    }
    
    else{
        init_ = pt2;
        end_  = pt1;
    }

    return;
}

/**
 * @brief Displays the start and end point coordinates of the line
 */
void line::printXY()
{
    std::cout<<init_.first<<" "<<init_.second<<" "<<end_.first<<" "<<end_.second<<std::endl;
}

/**
 * @brief Get one of the terminal points coordinate having lesser value of y.
 * 
 * @return std::pair<double, double> 
 */
std::pair<double, double> line::getOneEnd()
{
    return init_;
}

/**
 * @brief Get the terminal point having higher y-coordinate than the other terminal point
 * 
 * @return std::pair<double, double> 
 */
std::pair<double, double> line::getOtherEnd()
{
    return end_;
}

/**
 * @brief Can set init_ object directly using the input data arbitrarily.
 * 
 * @param tline 
 */
void line::setOneEnd(std::pair<double, double>& tline)
{
    init_ = tline;
}

/**
 * @brief Can set end_ object directly using the input data arbitrarily.
 * 
 * @param tline 
 */
void line::setOtherEnd(std::pair<double, double>& tline)
{
    end_ = tline;
}

/**
 * @brief Stores the status of the line while building a path. 
 * 
 * @return true 
 * @return false 
 */
bool line::isExplored()
{
    return checked_once_;
}