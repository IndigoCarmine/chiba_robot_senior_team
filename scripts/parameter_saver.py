#!/usr/bin/python python
##it will dump rosparam when a topic is published
import rospy
import rosparam
from chiba_robot_senior_team.srv import Saver
import time

def callback(data:Saver):
    if data.save_or_load:
        ##time stamp
        t = rospy.Time.now()
        t = time.strftime("%Y_%m_%d_%H_%M_%S", time.localtime(t.secs))
        ##save rosparam
        rosparam.dump_params("ros_dump_"+t+".yaml")
        rosparam.dump_params("ros_dump_latest.yaml")
    else:
        rosparam.load_file("ros_dump_latest.yaml")

    return

def main():
    rospy.init_node('parameter_saver')
    rospy.Service('parameter_saver', Saver, callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass