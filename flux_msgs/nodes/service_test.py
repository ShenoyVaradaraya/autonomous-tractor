#!/usr/bin/env python3
"""
Module to test the service call
"""
# Third-Party Imports
import rospy
from flux_msgs.srv import ModuleStatus, ModuleStatusResponse


class Node:
    """
    ROS Node for interfacing with the master script
    """
    def __init__(self):
        """
        Initialize the ROS related data
        """
        # Initialize node and log the data
        rospy.init_node('server_node')
        rospy.loginfo('Waiting for Service Request..')

        # Service
        rospy.Service('service_name', ModuleStatus, Node.handle_transformation_request)

        # Iterate over once request is received
        rospy.spin()

    @staticmethod
    def handle_transformation_request(request):
        """
        Function to Handle service request of coordinate transformation
        """
        response = ModuleStatusResponse()
        print(request)
        if request.component == True:
            response.status = True
        else:
            response.status = False
        return response


if __name__ == '__main__':
    """
    Main Function for this module
    """
    Node()
