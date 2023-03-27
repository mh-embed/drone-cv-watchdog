#!/usr/bin/python
import rospy
from std_msgs.msg import String, Bool
import sys, json

# GLOBAL VARIABLES
WATCHDOG_RATE = 100
CRITICAL_TIME = 500000000

# Node UID List
CV_UID_LIST = ["EMERGENCY"]


# Node Publisher Class
# Manages Publishing start / stop of a Single Node
class cv_node_publisher:
    def __init__(self, uid):
        self.uid = uid
        self.cv_selector_bit = rospy.Publisher('cv_selector/' + uid, Bool, queue_size=1)
        self.cv_selector_bit.publish(False)

    def start(self):
        self.cv_selector_bit.publish(True)

    def stop(self):
        self.cv_selector_bit.publish(False)

    def get_uid(self):
        return self.uid


# Node Manager Class
# Manages All Nodes and interacts with the main script
class cv_manager:
    def __init__(self):
        self.cv_nodes = []
        self.current_node_index = -1

        # Initialize Nodes, default to NOT start
        self.read_cv_uid_list()

    def read_cv_uid_list(self):
        for uid in CV_UID_LIST:
            self.cv_nodes.append(cv_node_publisher(uid))


    # To Be Implemented
    # Monitors Active Response From the Running CV Instance
    def ensure_cv_response(self):
        return

    # Determine whether the time to last published is over the limit
    # If over limit, start emergency control and hand back to manual 
    def is_over_limit(self, last_timestamp):
        now = rospy.get_rostime().to_sec()
        return (now - last_timestamp.to_sec() > CRITICAL_TIME)

    # To be implemented
    def emergency_response(self):
        return

    def update_cv(self, data):
        if self.current_node_index != -1:
            self.cv_nodes[self.current_node_index].stop()
        if data == "Manual":
            self.current_node_index = -1
        else:
            for node in self.cv_nodes:
                if node.get_uid() == data:
                    node.start()
                    return


# Main Driver of CV
def main():

    # Initialize the Node
    rospy.init_node('cv_watchdog')

    # Setup Publishing
    # Setup Manager for Each CV program
    manager = cv_manager()

    # Setup Subscribing
    # Setup Selector Bit 
    rospy.Subscriber('cv_manager/selector', String, manager.update_cv)

    # Setup Spin Rate
    rate = rospy.Rate(WATCHDOG_RATE)       # All CV Programs run at 30hz

    # Check CV Response and switch to manual if necessary
    while not rospy.is_shutdown():
        manager.ensure_cv_response()
        print("Node Running!")
        rate.sleep()


if __name__ == '__main__':
    main()
