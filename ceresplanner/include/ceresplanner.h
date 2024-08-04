#ifndef CERESPLANNER_H
#define CERESPLANNER_H

#include <opencv2/opencv.hpp>
#include "subregion.h"
#include "line.h"
#include <vector>
#include <fstream>
#include <map>
#include <algorithm>
#include <string>
#include "node.h"


enum class designPattern{OneWayPattern, MultiWayOneSkip, MultiWayTwoSkip};// Available options

class ceresPlanner
{
    public:             

        // test each modules now using the input and the output
        cv::Mat setInput(cv::Mat&); 

        // detecting the corners on the farm boundary to define farm as a polygon
        std::vector<cv::Point> contourMapping(cv::Mat&);     
        
        // obtaining convex region overlapped over original concavr farm 
        //std::vector< std::vector<cv::Point> > convexHullDriver(std::vector< std::vector<cv::Point> >&, cv::Mat&);         
        
        // finding the optimal direction  
        //double optimalDirection(std::vector< std::vector<cv::Point> >&);    
        
        void dump_A(std::ofstream&, std::vector<cv::Point>&);
        void dump_A(std::ofstream&, subregion&);
        void dump_A(std::ofstream&, std::vector<std::pair<int, int>>&);
        void dump_A(std::ofstream&, std::vector<std::vector<line>>&);
        void dump_A(std::ofstream&, std::vector<Node>&);

        subregion straightPathDraw(std::vector<cv::Point>&, int&, double&);
        subregion headLandCreator(subregion&, int&);
        std::vector<int> gradientBreakinY(std::vector<int>&, double&);
        // tracing the path throught the region
        //lineSweep();                   
        
        // joining the lines based on the skip requirments
        std::vector<std::pair<int,int>> pathPlanning(subregion&, int&);
        std::vector<std::pair<int,int>> skipTurnDesignedPath(subregion&, designPattern&);          
        // print paths in each subregion
        // outputSubRegionPath(); 

    private:

        cv::Mat InputBoundaryData_;
        //std::vector<std::pair<int, subregion> > tracedRegion_;
        std::vector<std::pair<int, int> > plannedPath_;
    
        //static int subRegionId_;
};

#endif