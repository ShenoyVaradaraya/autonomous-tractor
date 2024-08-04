#!/usr/bin/env python3
"""
Module to request for a service from a server
"""
# Third-Party Imports
import rospy
from flux_msgs.srv import ModuleStatus



def request_hardware_diagnotics_status():
    """
    Function to send the coordinates of the corner points of the vehicle in the map frame
    """
    rospy.wait_for_service('/service_name', timeout=5)
    try:
        hardware_diagnostics = rospy.ServiceProxy('/service_name', ModuleStatus)
        _data = ModuleStatus()

        # _data.component = list(filter(lambda x: x, 'hardware'))
        # _data.component = "hardware"
        _data.component = True
        print(_data.component, type(_data.component))
        response = hardware_diagnostics(_data)
        print(response)
        # response = hardware_diagnostics("hardware")
        return response.status
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)
        return None


if __name__ == '__main__':
    rospy.init_node('client')
    rospy.loginfo('Client ready to request for the service')

    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        response = request_hardware_diagnotics_status()
        print(response)
        rate.sleep()
