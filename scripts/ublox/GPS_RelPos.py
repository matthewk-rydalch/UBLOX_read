import rospy
import sys
import ublox.WaypointConvert as wp
import ublox
import PositionVelocityTime.msg as posveltime

class GPS_RelPos:

    def __init__(self):
        rospy.init_node('GPSRelPos')
        master = rospy.get_param('~master')
        servant = rospy.get_param('~servant')

        self.mastersub = rospy.Subscriber(master, posveltime, callback=self.master_CB)
        self.servantsub = rospy.Subscriber(servant, posveltime, callback=self.servant_CB)

    def master_CB(self, msg):
        print('Master Callback')

    def servant_CB(self, msg):
        print('Servant Callback')
        
if __name__ == "__main__":
    
    thing = GPS_RelPos()

    rospy.spin()