#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

#Nose tip position initialization
xData = []
yData = []
how_many_points = rospy.get_param("how_many_plot", 10)
# Initialize the plot
fig, ax = plt.subplots()
line_path, = ax.plot([], [], 'go-', label="Nose path")
line_tip, = ax.plot([], [], 'ro', label="Current nose position", markersize=8)

ax.set_xlim(0, 640)
ax.set_ylim(0, 480)
ax.invert_yaxis()
ax.set_xlabel("X")
ax.xaxis.set_label_position('top')
ax.xaxis.tick_top()
ax.set_ylabel("Y")
ax.set_title("Nose tip position")
ax.grid(True)
ax.legend()

# Callback function to update the plot with new nose tip coordinates
def callbackLimitPoints(msg):
    xData.append(msg.x)
    yData.append(msg.y)
    if len(xData) > how_many_points: #Limit the number of points #type: ignore
        xData.pop(0)
        yData.pop(0)

def callbackNoLimitPoints(msg):
    xData.append(msg.x)
    yData.append(msg.y)


# Animation function to update the plot
def animate(i):
    if not xData:
        return
    line_path.set_data(xData[:-1], yData[:-1])
    line_tip.set_data(xData[-1:], yData[-1:])
    ax.relim()
    ax.autoscale_view()

def main():
    rospy.init_node("nose_plotter", anonymous=True)
    if how_many_points > 0: #type: ignore
        rospy.Subscriber("nose_coordinates", Point, callbackLimitPoints)
    else:
        rospy.Subscriber("nose_coordinates", Point, callbackNoLimitPoints)

    rospy.loginfo("Nose plotter node started")
    ani = FuncAnimation(fig, animate, interval=100)
   
    plt.show()
   
if __name__ == "__main__":
    main()
