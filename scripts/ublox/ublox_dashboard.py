#!/usr/bin/env python
import rospy
from ublox.msg import RelPos as rp
import rospy
import matplotlib.animation as animation
import matplotlib.pyplot as plt

class Dashboard:

    def __init__(self):

        self.data_flag = False

        rospy.init_node('dashboard')

        self.fig, self.axes = plt.subplots(nrows=2, ncols=3)

        self.fig.suptitle(rospy.get_param('/dash/dashboard/title', default='Default'))

        self.axes[0][0].set_title('RTK North vs Time')
        self.axes[0][0].set_xlabel('Time (s)')
        self.axes[0][0].set_ylabel('N (m)')

        self.axes[0][1].set_title('RTK East vs Time')
        self.axes[0][1].set_xlabel('Time (s)')
        self.axes[0][1].set_ylabel('E (m)')

        self.axes[0][2].set_title('RTK Down vs Time')
        self.axes[0][2].set_xlabel('Time (s)')
        self.axes[0][2].set_ylabel('D (m)')

        self.axes[1][0].set_title('RTK Flags vs Time')
        self.axes[1][0].set_xlabel('Time (s)')
        self.axes[1][0].set_ylabel('Flags')

        self.axes[1][1].set_title('RTK North vs RTK East')
        self.axes[1][1].set_xlabel('E (m)')
        self.axes[1][1].set_ylabel('N (m)')

        relpostopic = rospy.get_param('/dash/dashboard/relpostopic')
        self.roverRelPosSub = rospy.Subscriber(relpostopic, rp, self.relposCB)

        self.ani = animation.FuncAnimation(self.fig, self.update_plots)

        plt.show()
        

    def update_plots(self, i):
        print('update plots')
        if(self.data_flag):
            self.axes[0][0].plot(self.relpos.header.stamp.to_sec(), self.relpos.relPosNED[0]+self.relpos.relPosHPNED[0], 'b*')
            self.axes[0][1].plot(self.relpos.header.stamp.to_sec(), self.relpos.relPosNED[1]+self.relpos.relPosHPNED[1], 'b*')
            self.axes[0][2].plot(self.relpos.header.stamp.to_sec(), self.relpos.relPosNED[2]+self.relpos.relPosHPNED[2], 'b*')
            self.axes[1][0].plot(self.relpos.header.stamp.to_sec(), self.relpos.flags, 'b*')
            self.axes[1][1].plot(self.relpos.relPosNED[1]+self.relpos.relPosHPNED[1], self.relpos.relPosNED[0]+self.relpos.relPosHPNED[0], 'b*')


    def relposCB(self, msg):
        self.data_flag=True
        print('relposCB')
        self.relpos = msg



if __name__ == '__main__':
    thing = Dashboard()

    rospy.spin()

