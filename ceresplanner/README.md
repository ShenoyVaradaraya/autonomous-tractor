# Ceres Planner
### Description
The developed project can generate suitable ploughing patterns on any regularly shaped farmland with maximum coverage leading to maximum gains for the user.
_Input_ 
1. An image of the farmland with clearly visible contour is an ideal input for the developed planner.
2. The start location/pose of the vehicle is given by the localisation module.
3. Different headland space requirements based on the type of implements to be used with the tractor.
        
![example input of a farmland](/images/ceres_input.png)

### Developed Methodology
1. At the start, one end of one of the extreme lines is chosen and then we traverse on the same line to the other end.
2. On the other end, planner searches for the nearest end of a neighbouring line, thus a path connecting the current line to the next line is drawn at the found nearest end.
3. Depending on the required skip turn, selection of neighbouring lines is done after skipping the intermediate contours. Skipped line extreme points are pushed on a stack.
4. Once we reach the other extreme, points from the stack are popped and are included in the planned path.

### Planner Results
![Path on input farmland](/images/planner_output_arbitrary_field.png)
![Path on rectangular farmland](/images/planner_output_rectangular_farm.png)
![Path on hexagonal farmland](/images/planner_output_hexagonal.png)
![Path on octagonal farmland](/images/planner_output_octagonal_farm.png)

### Steps for Obtaining planner Output for different input cases
 ```
g++ -std=c++11 -Wall -Wextra -g ceres_main.cpp ceresPlanner.cpp line.cpp subregion.cpp -o run
or use a cMakeList to compile and execute.
 ```
### Brief Description of the developed objects under FMS
For detailed description refer to the [documentation](). {Use class names from the table below to access the documented html page}.
|SI.  |  Sub Tasks                        | Summary                           |
|-----|----------------------------------|------------------------------------|
|1.   |ceres_main| Takes input as farmland image and calls appropriate functions of the ceresPlanner class to process input image.|
|2.   |ceresplanner class| Processes the image and plans coverage path on the input map.|
|3.   |line class | Constituent of the planned path, since the planned path is composed of many such straight lines.|
|4.   |subregion class  | Convex map region or convex sub-region of an input concave map.|

### Diagrams and Flow-Charts
![Ceres Planner Flow Chart](/images/ceresPlanner_FC.png)


### Future Work
1. POC testing due.
2. Modifications of the POC for non-convex farmlands.

### Author
Kumar Rahul Dinkar.

### License
Copyright (c) 2022 FLUX AUTO.