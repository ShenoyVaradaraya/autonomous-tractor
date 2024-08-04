/* Author: Kumar Rahul Dinkar */
#ifndef LINE_H
#define LINE_H

#include <vector>

class line
{
    public:
        line(const double);// input is the id
        void setInput(const std::pair<double, double>&, const std::pair<double, double>&);//start and end points
        double getTheta();
        double getLength();
        void printXY();// prints the start and endpoint of the line on screen
        // OneEnd function deals with the point on the line segment closer 
        // to the y-axis, while OtherEnd operates on the point farther from y-axis
        std::pair<double, double> getOneEnd();
        void setOneEnd(std::pair<double, double>&);
        std::pair<double, double> getOtherEnd();
        void setOtherEnd(std::pair<double, double>&);
        // returns true if the line is explored or else false
        bool isExplored();

    private:
        std::pair<double, double> init_;// start coordinates
        std::pair<double, double> end_;// end coordinates
        bool checked_once_, bifurcation_point_; 
        double optimalDirection_;
};
#endif