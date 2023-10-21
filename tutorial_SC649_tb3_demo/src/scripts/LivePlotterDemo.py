#! /usr/bin/env python3

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
#from detect_gaps import gaps,range_smoother,R_max_finder,waypoint_finder

global xdata,ydata,line,scan,Gaps,R_max_set,line_raw

global prev_gaps
prev_gaps=[]

def callback(msg):
    global scan,Gaps,R_max_set,prev_gaps
    scan = msg

def main_live_plotter():

    rospy.init_node('Live_Plotter')

    global xdata,ydata,line,scan,line_raw
    sub = rospy.Subscriber('/tb3_4/scan',LaserScan,callback)

    #while(len(prev_gaps)==0):
        #print("First Time Gap Not detected")
        #continue
    
    fig, ax = plt.subplots(figsize=(100, 100), subplot_kw={'projection': 'polar'})
    fig2, ax2 = plt.subplots(figsize=(100,100),subplot_kw={'projection': 'polar'})
    line, = ax.plot([], [], lw = 2)
    line_raw, = ax2.plot([], [], lw=2)

    #line = ax.scatter([], [])
    # initializing empty values
    # for x and y co-ordinates
    xdata, ydata = [], []

    while not rospy.is_shutdown():

        # calling the animation function	
        #anim = animation.FuncAnimation(fig, animate,init_func = init,frames = 5000,interval = 20,blit = True)
        anim_2 = animation.FuncAnimation(fig2, animate_2, init_func = init_raw_sensor, frames = 5000, interval = 20,blit = True)
        #plt.clf()
        # saves the animation in our desktop
        #anim.save('growingCoil.mp4', writer = 'ffmpeg', fps = 30)
        #xdata = np.linspace(scan.angle_min,scan.angle_max,num = len(scan.ranges))
        #ydata = scan.ranges
        #line.set_data(xdata, ydata)

        #plt.pause(0.5)
        plt.show()


def init_raw_sensor():
    global line_raw
    line_raw.set_data([],[])
    return line_raw,


# what will our line dataset
# contain?
def init():
    global line
    line.set_data([], [])
    return line,



def animate_2(i):

    global xdata,ydata,line,scan,Gaps,R_max_set
    xdata = []
    ydata = []
	# appending values to the previously
	# empty x and y data holders
    #for i in range(len(Gaps)):
        #for j in range(2):
            #xdata.append(scan.angle_min + Gaps[i][j]*scan.angle_increment)
            #ydata.append(scan.ranges[Gaps[i][j]])
    
    
    ydata = range_smoother(scan.ranges,scan.range_min,scan.range_max)
    #ydata = R_max_finder(scan.ranges)
    ydata = scan.ranges
    xdata = np.linspace(scan.angle_min,scan.angle_max,num = len(ydata))
    if(len(ydata)>0):
        line_raw.set_data(xdata, ydata)
    #line.set_offsets([[xdata],[ydata]])

    return line_raw,


if __name__ == '__main__':
    try:
        main_live_plotter()
    except rospy.ROSInterruptException:
        pass