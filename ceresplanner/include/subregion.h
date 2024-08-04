/* Author: Kumar Rahul Dinkar */
#ifndef SUBREGION_H
#define SUBREGION_H

#include "line.h"

class subregion
{
    public:
        subregion(const int);
        void insertLines(const int, const line&);
        int lineCount();
        int checkID();
        void printLines();
        std::vector<line> getIterRegion();
        void setIterRegion(std::vector<line>&);
        
    private:
        std::vector<line> path_;
        int id_;
};
#endif