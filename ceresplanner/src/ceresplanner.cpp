/**
 * @file ceresplanner.cpp
 * @author Kumar Rahul Dinkar
 * @brief 
 * @version 0.1
 * @date 2022-01-10
 * 
 * @copyright Copyright (c) 2022 FLUX AUTO
 * 
 */

#include "ceresplanner.h"
#include "subregion.h"
#include <iostream>
#include <stack>

/**
 * @brief Converts the inout image into binary image
 * 
 * @param image input image of the farm
 * @return cv::Mat binary image
 */
cv::Mat ceresPlanner::setInput(cv::Mat& image)
{
    if(!image.data)
    {
        std::cout<<"Could not open file" << std::endl;
        return image;
    }

	else
	{
		InputBoundaryData_ = image;
	}

    //std::cout<<"I am inside setInput"<<std::endl;
    cv::Mat gray, threshold_output;
	cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY); // convert to grayscale
    cv::threshold(gray, threshold_output, 127, 255, cv::THRESH_BINARY);

	//std::cout<<"after threshold"<<std::endl;

	cv::imwrite("setInputfile.png", threshold_output);

	//std::cout<<"before return"<<std::endl;
    //std::cout<<"maze_plain"<<threshold_output.type()<<std::endl;

	return threshold_output;

}

/**
 * @brief Extracts the boundary coordinates from binary image
 * 
 * @param threshold_output binary image
 * @return std::vector<cv::Point> boundary coordinates (pixel)
 */
std::vector<cv::Point> ceresPlanner::contourMapping(cv::Mat& threshold_output)
{
	int end_x = threshold_output.cols - 1;  // x is col
	int end_y = threshold_output.rows - 1;  // y is row

	std::vector<cv::Point> boundaryCoordinates;
	int _cnt = 0;
	int _imgCount = 0;

	for(int i = 0; i < end_y; i++)      // i is row
	{
		//cv::Mat nr = image.row(i);
		for(int j = 0; j < end_x; j++)  // j is col
		{
			_imgCount++;
			if(threshold_output.at<uchar>(i,j) ==0 )
			{
				//std::cout<<" "<<threshold_output.row(i).col(j)<<"i:"<<i<<"j:"<<j<<std::endl;
				cv::Point temp;
				temp.x = j;
				temp.y = i;
				boundaryCoordinates.push_back(temp);
				_cnt++;
				//std::cout<<std::endl;
			}
		}
	
	}	

	//std::cout<<"Total count of the container: "<<_cnt<<"out of total image cell: "<<_imgCount
	//<<"size of the boundaries"<<boundaryCoordinates.size()<<std::endl;
/*
	std::vector< std::vector<cv::Point> > contours; // list of contour points
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(threshold_output, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
	std::cout<<"I am here in the contour mapping"<<std::endl;
	std::cout<<contours.size()<<std::endl;
	

	for(int i=0; i<contours.size();i++)
	{
		std::cout<<"index: "<<i<<" "<<contours[i].size()<<" :"<<std::endl;
		for (int j=0; j< contours[i].size(); j++)
		{
			std::cout<<contours[i][j]<<" ";

		}
		std::cout<<std::endl;

	}
*/
	return boundaryCoordinates;
}

/**
 * @brief Filters the y-values so as to keep only those points at same 'x' which are separated by
 *        an interval of more than the gradientThreshold
 * 
 * @param y_vals contains all the y_values lying at the same x
 * @param gradientThreshold in pixels.
 * @return std::vector<int> 
 */

std::vector<int> ceresPlanner::gradientBreakinY(std::vector<int>& y_vals, double& gradientThreshold)
{
	// input vector must be of size greater than equal to 4
	std::vector<int> temp;
	double diff;

	temp.push_back(y_vals[0]);

	for(int i = 1; i < y_vals.size(); i++)// ASSUMPTIONS HERE
	{
		diff = y_vals[i]- y_vals[i-1];

		if(diff > gradientThreshold)
		{
			temp.push_back(y_vals[i]);
		}
		
	}
	
	if( temp[temp.size()-1] != y_vals[y_vals.size()-1] )
	{
		diff = y_vals[y_vals.size()-1] - temp[temp.size()-1];
		if(diff > gradientThreshold)
			temp.push_back(y_vals[y_vals.size()-1]);
	}

//temp.push_back(y_vals[y_vals.size()-1]);
/////////////////////////////////DEBUG/////////////////////////////
/*	std::cout<<"||DEBUG INPUT_YVAL||";
	for(int i=0; i<y_vals.size();i++)
	{
		std::cout<<y_vals[i]<<" ";
	}
	
	std::cout<<"||DEBUG OUTPUT_YVAL||"<<std::endl;
	for(int i=0; i<temp.size();i++)
	{
		std::cout<<temp[i]<<" ";
	}
*/
///////////////////////////////////////////////////////////////////

	return temp;
}

/**
 * @brief 1. contourPoints is processes to store the y-valaues corresponding to each x in the 
	         procesContainer_ map, where key is the x-value and y are the values of the points
	         having the same x.
 * 		  2. Lines are generated vertically at given x in alternative intervals of y 
 * 
 * @param contourPoints contourPoints contains all the farm boudary points. 
 * @param parking_width separation width between to parallel vertical lines 
 * @param gradientThreshold acceptable separation between y-values to be considered for drawing a line
 * @return subregion 
 */

subregion ceresPlanner::straightPathDraw(std::vector<cv::Point>& contourPoints, int& parking_width, double& gradientThreshold)
{
	// order it in term of increasing x.
	std::map<int, std::vector<int> > processContainer_;
	

	for(int i = 0; i < contourPoints.size(); i++)
	{
		auto tempf = processContainer_.find(contourPoints[i].x);

		if(tempf != processContainer_.end())
		{
			tempf->second.push_back(contourPoints[i].y);
		}
		else
		{
			std::vector<int> tempV;
			tempV.push_back(contourPoints[i].y);
			processContainer_[contourPoints[i].x] = tempV;
		}

	}

	subregion master(0);
    // start drawing the straight line at min_x+h point at 
	// each increment of h, till the choosen point is less than max_x
	
	int xMark_ = 0;
	for(std::map<int,std::vector<int>>::iterator iter = processContainer_.begin(); iter != processContainer_.end(); ++iter)
	{
		if (xMark_ == 0)//THIS IS AN ASSUMPTION
		{
			// setxMark_ here;
			xMark_ = iter->first + parking_width;
			continue;

		} 
        
		if(iter->first == xMark_)
		{
			line tLine(0);
	     	
			int x_val = iter->first;
			std::vector<int> y_vals = iter->second;
			// find the max and min y with the gradient change greater than 5
			// if 2 ys found then 1 line is found
			// if 4 ys found then 2 line is found
			//step::1 let us find the max
              
			std::sort(y_vals.begin(), y_vals.end());

			// find the gradient in the y_vals vector
			std::vector<int> newY_vals;

			// Lines are generated vertically using the below code at given x 	
			if(y_vals.size() >= 4)
			{
				newY_vals = gradientBreakinY(y_vals, gradientThreshold);
				int i = 0;

				while(i < newY_vals.size())
				{
					int y_max = newY_vals[i+1];
					int y_min = newY_vals[i];
					tLine.setInput(std::make_pair(x_val, y_min), std::make_pair(x_val, y_max));
        			master.insertLines(0, tLine);
					i += 2;
				}
			}

            else
			{
				// now we will make lines using the alternate interval in ys
				// this function should return y_values around those gradient
				// here we will iterate over the returned vector to push new lines
				// in an alternate fashion.
				int y_max = y_vals[y_vals.size()-1];
				int y_min = y_vals[0];

				tLine.setInput(std::make_pair(x_val, y_min), std::make_pair(x_val, y_max));
        		master.insertLines(0, tLine);
			}

			xMark_ = iter->first + parking_width;

		}
		//ignore value
		//Value v = iter->second;
	}

	// store each of the lines in subregion
	return master; 	
}

/**
 * @brief The generated lines on the farm map is trimmed near to farm boundaries
 *        so as create sufficient space for vehicle to turn.  
 * 
 * @param plan  subregion object storing the generated lines on the farmland.
 * @param headLandSpace space needed between line termination and the map boundary.
 * @return subregion 
 */

subregion ceresPlanner::headLandCreator(subregion& plan, int& headLandSpace)
{	
	std::vector<line> path = plan.getIterRegion();

	for(int k = 0; k < path.size(); k++)
	{
		double tempxOne = path[k].getOneEnd().first;
		double tempyOne = path[k].getOneEnd().second + headLandSpace;

		double tempxOther = path[k].getOtherEnd().first;
		double tempyOther = path[k].getOtherEnd().second - headLandSpace;

		std::pair<double, double> tempOne = std::make_pair(tempxOne, tempyOne);
		std::pair<double, double> tempOther = std::make_pair(tempxOther, tempyOther);

		// line termination points are updated with headLand space.
		path[k].setOneEnd(tempOne); 
		path[k].setOtherEnd(tempOther);

	}

	// sets the updated paths in the subRegion/map.
	plan.setIterRegion(path);
	return plan;

}
/*
std::vector< std::vector<cv::Point> > ceresPlanner::convexHullDriver(std::vector< std::vector<cv::Point> >& contours, cv::Mat& threshold_output)
{
	// create hull array for convex hull points
	std::vector< std::vector<cv::Point> > hull(contours.size());

	// A C++ program to find convex hull of a set of points. Refer

	// To find orientation of ordered triplet (p, q, r).
	// The function returns following values
	// 0 --> p, q and r are collinear
	// 1 --> Clockwise
	// 2 --> Counterclockwise

	cv::Point points[] = {{0, 3}, {2, 2}, {1, 1}, {2, 1},
					{3, 0}, {0, 0}, {3, 3}};
	int n = sizeof(points)/sizeof(points[0]);
	convexHull(contours, hull);

	return hull;
}
*/

/**
 * @brief Stores the binary image data into file
 * 
 * @param os file stream object 
 * @param data image data
 */
void ceresPlanner::dump_A(std::ofstream& os, std::vector<cv::Point>& data)
{

	for(int j=0 ;j<data.size();j++)
	{
		os<<data[j].x<<" "<<data[j].y;
		os<<'\n';
	}
		
}

/**
 * @brief Stores the drawn vertical parallel lines coordinates in the output file 
 * 
 * @param os file stream object
 * @param temp subregion object
 */
void ceresPlanner::dump_A(std::ofstream& os, subregion& temp)
{
	std::vector<line> region = temp.getIterRegion();

	for(int j=0 ;j < region.size();j++)
	{
		os<<region[j].getOneEnd().first<<" "<<region[j].getOneEnd().second<<'\n';
		os<<region[j].getOtherEnd().first<<" "<<region[j].getOtherEnd().second;
		
		os<<'\n';
	}
		
}

/**
 * @brief Stores each line terminal points in the sequence, they can be visited 
 *        as planned by the path planner in an output file.
 * 
 * @param os file stream object.
 * @param temp planned path
 */
void ceresPlanner::dump_A(std::ofstream& os, std::vector<std::pair<int, int>>& temp)
{
	for(int j = 0 ;j < temp.size();j++)
	{
		os<<temp[j].first<<" "<<temp[j].second<<'\n';
	}
		
}
void ceresPlanner::dump_A(std::ofstream& os, std::vector<Node>& temp)
{
	for(int j = 0 ;j < temp.size();j++)
	{
		os<<temp[j].x<<", "<<temp[j].y<<", "<<temp[j].orien<<'\n';
	}
		
}

/**
 * @brief Stores each cluster of lines in an output file.
 * 
 * @param os file stream object
 * @param temp clustered  lines at each x
 */
void ceresPlanner::dump_A(std::ofstream& os, std::vector<std::vector<line>>& temp)
{
	for(int j=0 ; j < temp.size(); j++)
	{
		for(int k=0;k<temp[j].size();k++)
		{
			os<<temp[j][k].getOneEnd().first<<" "<<temp[j][k].getOneEnd().second<<" "
			<<temp[j][k].getOtherEnd().first<<" "<<temp[j][k].getOtherEnd().second;
			//os<<temp[j].getOtherEnd().first<<" "<<temp[j].getOtherEnd().second;
			os<<"\n";
		}
		os<<"\n";
	}
		
}

/**
 * @brief 1. Clusters all the lines for each x in the subregion class line_container_
 *        2. then joins the nearest endpoints of the neighboring lines together to 
 *           join all the lines forming a global coverage path.  
 * 
 * @param line_container_ Contains all the disconnected parallel lines drawn on the farmland
 * @param key Type of connections needed among the vertical parallel lines.
 * @return std::vector<std::pair<int,int>> planned path is returned
 */
std::vector<std::pair<int,int>> ceresPlanner::pathPlanning(subregion& line_container_, int& key)
{
	std::vector<line> tempPlan =  line_container_.getIterRegion();

	// input is this vector of lines
	// output is vector containing sequence of points to be taken as path 
	// for coverage planning
	// I need data container as vector of vector lines
	std::vector<std::vector<line>> ws_;
	//////////////////////////////////////////PREPARING THE WS_ DATA///////////////////////////////
	int xIterPast = tempPlan[0].getOneEnd().first;
	int xIterCurr;
	std::vector<line> tempIter;
 
	// output is [ws_] which stores cluster of lines at same x in one container
	for(int i = 0; i < tempPlan.size(); i++)
	{
		xIterCurr = tempPlan[i].getOneEnd().first;
		
		if(xIterCurr == xIterPast)
		{
			tempIter.push_back(tempPlan[i]);
		}

		else
		{
			ws_.push_back(tempIter);
			tempIter.clear();
			tempIter.push_back(tempPlan[i]);
			xIterPast = tempPlan[i].getOneEnd().first;
		}
			
	}
    // now ws stores each cluster of lines so there will be one cluster for each x 
	// for the last element

	ws_.push_back(tempIter);
	/////////////////////////////////DEBUG/////////////////////////////////////////////////////////
    std::ofstream tpoutfile("workspace.txt");
    dump_A(tpoutfile, ws_);
	
	///////////////////////////////////////////////////////////////////////////////////////////////
	// 1 stack container
	// /std::stack<std::pair<int, int>> auxiliary_;
	
	// initialize the plannedPath_ container with the first line and the stack with its 
	// auxiliary line,
	plannedPath_.push_back(ws_[0][0].getOneEnd());    // pushback a pair of points in the container
	plannedPath_.push_back(ws_[0][0].getOtherEnd());  // pushback a pair of points in the container

	int iter = 0;
	bool fwd = true;
	int fwd_i;
	int rev_i = ws_.size() - 1;

	//while(plannedPath_.size() <= tempPlan.size() * 2)
	for(int fwd_i = 1; fwd_i < ws_.size(); fwd_i++)
	{
		// with each new line, we first add a line connecting the current point to the nearest point
		// in the skipped line. (use the last point in the planned path, add first the nearest
		// point in the next line then add the farthest point)
		if(fwd)
		{
			// check distance between last pt. in the path with the next line
			int oneEnd_diff =   std::abs(plannedPath_[plannedPath_.size() - 1].second - ws_[fwd_i][0].getOneEnd().second);
			int otherEnd_diff = std::abs(plannedPath_[plannedPath_.size() - 1].second - ws_[fwd_i][0].getOtherEnd().second);

			if(oneEnd_diff <= otherEnd_diff)
			{
				plannedPath_.push_back(ws_[fwd_i][0].getOneEnd());    // pushback a pair of points in the container
				plannedPath_.push_back(ws_[fwd_i][0].getOtherEnd());  // pushback a pair of points in the container
				
			}

			else
			{
				plannedPath_.push_back(ws_[fwd_i][0].getOtherEnd());  // pushback a pair of points in the container
				plannedPath_.push_back(ws_[fwd_i][0].getOneEnd());  // pushback a pair of points in the container
				//CHECK IF MORE THAN ONE LINES AT SAME X

			}

			//fwd_i++;
		}
		//FILL THE STACK ELEMENTS------???????????

		//else
		//{

		//}
		// for generalising check if the planned path number of points is same as 
		// avaiable in ws, else now traverse the list in backward direction 
		// iter = iter + key; 
	}

	// skipped point is choosen if its available and is unexplored, else start checking from
	// immediate next unexplored lines. 
	// once at the end of the vector of lines, return from there so now look for 
	return plannedPath_;

} 

/**
 * @brief Configures and calls the planner using appropriate key based on the
 *        requested design pattern on the farmland.
 *        
 * @param line_container_ contains all the disconnected parallel lines drawn on the farmland
 * @param design type of connections needed among the vertical parallel lines.
 * @return std::vector<std::pair<int,int>> 
 */
std::vector<std::pair<int,int>> ceresPlanner::skipTurnDesignedPath(subregion& line_container_, designPattern& design) 
{
	std::vector<std::pair<int, int>> fPath_;
	int key = 10;// initial value

	switch(design)
	{
		case designPattern::OneWayPattern: key = 0;
			break;	
		case designPattern::MultiWayOneSkip: key = 1;
			break;
		case designPattern::MultiWayTwoSkip: key = 2;
			break;
		default: std::cout<<"You got to select one pattern for further proccessing. quitting"<<std::endl;
	};

	if(key != 10)
		fPath_ = pathPlanning(line_container_, key);

	return fPath_;
}            