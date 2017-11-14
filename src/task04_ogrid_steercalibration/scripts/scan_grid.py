#!/usr/bin/env python

# --- imports ---
import rospy
import numpy as np
import sys
import matplotlib.pyplot as plt
# import shapely
# import math
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan

CELL_OCCUPIED = 100
CELL_FREE = 0
CELL_UNKNOWN = -1

# TODO add beginign and ends, OCCUPIED after LINE
class free_line_drawer(object):

    def __init__(self, occupancy_grid):
        self.resolution = occupancy_grid.info.resolution

    def horizontal(self, x1, x2, y):
        if x1>x2:
            x1, x2 = x2, x1

        x_cur = x1
        while x_cur <= x2:
            set_cell(x_cur, y, CELL_FREE)
            x_cur += self.resolution
    
    def vertical(self, x, y1, y2):
        if y1>y2:
            y1, y2 = y2, y1

        y_cur = y1
        while y_cur <= y2:
            set_cell(x, y_cur, CELL_FREE)
            y_cur += self.resolution
    
    def diagonal(self, x1, y1, x2, y2):
        if x1>x2:
            x1, x2 = x2, x1
            y1,y2 = y2,y1

        length = x2 - x1
        inc = 0.0
        
        if y1 < y2:
            while inc <= length:
                set_cell(x1+inc, y1+inc, CELL_FREE)
                inc += self.resolution
        else:
            while inc <= length:
                set_cell(x1+inc, y1-inc, CELL_FREE)
                inc += self.resolution
            
    
    def set_free_line(self, x1, y1, x2, y2):
        if x1==x2 and y1==y2:
            return
        
        if x1==x2:
            self.vertical(x1, y1, y2)
        elif y1==y2:
            self.horizontal(x1, x2, y1)
        elif abs(x1-x2)==abs(y1-y2):
            self.diagonal(x1, y1, x2, y2)
        else:
            dx = x2-x1
            dy = y2-y1

            #beggining? x1,y1,FREE?
            
            if abs(dx) > abs(dy):
                if x1 > x2:
                    x1,x2=x2,x1
                    y1,y2=y2,y1
                
                slope = dy/dx
                slope *= self.resolution

                y_cur = y1
                x_cur = x1+self.resolution
                while x_cur<x2:
                    y_cur += slope
                    set_cell(x_cur, y_cur, CELL_FREE)
                    x_cur += self.resolution
            else:
                if y1>y2:
                    x1,x2=x2,x1
                    y1,y2=y2,y1
                
                slope = dx/dy
                slope *= self.resolution

                x_cur=x1
                y_cur=y1+self.resolution
                while y_cur<y2:
                    x_cur+=slope
                    set_cell(x_cur, y_cur, CELL_FREE)
                    y_cur+=self.resolution

# --- definitions ---
def resetGrid():
    global occupancy_grid
    
    w = occupancy_grid.info.width
    h = occupancy_grid.info.height
    res = occupancy_grid.info.resolution

    width_range = xrange(w)
    height_range = xrange(h)
    # set all values to "FREE"
    occupancy_grid.data = [CELL_UNKNOWN]*(h*w)
    # for x in width_range:
    #     for y in height_range:
    #         occupancy_grid.data[x+y] = CELL_UNKNOWN

# to a given cartesian x,y coordinate, mark the corresponding cell in the grid as "OCCUPIED"
def set_cell(x,y, cell_info):
    global occupancy_grid

    res = occupancy_grid.info.resolution
    x_scaled = (x * 1.0 / res) + occupancy_grid.info.width/2
    y_scaled = (y * 1.0 / res) + occupancy_grid.info.height/2

    if x_scaled >= occupancy_grid.info.width or x_scaled < 0 or y_scaled >= occupancy_grid.info.height or y_scaled < 0:
        return

    offset = (int(round(x_scaled)) - 1) * occupancy_grid.info.height
    occupancy_grid.data[int(offset) + int(round(y_scaled) - 1)] = cell_info#100


def scanCallback(scan_msg):
    # rospy.loginfo('CB!')
    global occupancy_grid

    resetGrid()
    drawer = free_line_drawer(occupancy_grid)

    # convert scan measurements into an occupancy grid
    rospy.loginfo(len(scan_msg.ranges))   
    rospy.loginfo(scan_msg)
    # rospy.loginfo(occupancy_grid) 

    angle_min = scan_msg.angle_min
    angle_cur = angle_min
    angle_max = scan_msg.angle_max
    angle_inc = scan_msg.angle_increment
    range_min = scan_msg.range_min
    range_max = scan_msg.range_max
    h = occupancy_grid.info.height
    w = occupancy_grid.info.width
    d_max = max(h, w)*occupancy_grid.info.resolution
    
    obstacle_x = []
    obstacle_y = []
    in_obstacle = False
    #d==vzdalenost
    for d in scan_msg.ranges:
        # found_obstacle = True
        if d > range_max or d < range_min:
            # found_obstacle = False
            d = d_max
            if in_obstacle:
                #TODO Fill last found
                
                in_obstacle = False
                obstacle_x = []
                obstacle_y = []
            # angle_cur += angle_inc
            # continue

        in_obstacle = True
        # if d != np.inf: 
        #     print d
        deg = np.rad2deg(angle_cur)
        x = np.sin(angle_cur)*d
        y = np.cos(angle_cur)*d #minus?
        #print "%s: [%s;%s]" % (deg, x, y)

        #try add lol
        drawer.set_free_line(0.0,0.0,x,y)
        set_cell(x, y, CELL_OCCUPIED)
        obstacle_x.append(x)
        obstacle_y.append(y)

        angle_cur += angle_inc

    rospy.loginfo("publishing occup. grid")
    pub_grid.publish(occupancy_grid)


# --- main ---
rospy.init_node("scan_grid")

# init occupancy grid
occupancy_grid = OccupancyGrid()
occupancy_grid.header.frame_id = "laser"
occupancy_grid.info.resolution = 0.01#None # in m/cell

# width x height cells
occupancy_grid.info.width = 1000#None
occupancy_grid.info.height = 1000#None

# origin is shifted at half of cell size * resolution
occupancy_grid.info.origin.position.x = int(-1.0 * occupancy_grid.info.width / 2.0) * occupancy_grid.info.resolution
occupancy_grid.info.origin.position.y = int(-1.0 * occupancy_grid.info.height / 2.0) * occupancy_grid.info.resolution
occupancy_grid.info.origin.position.z = 0
occupancy_grid.info.origin.orientation.x = 0
occupancy_grid.info.origin.orientation.y = 0
occupancy_grid.info.origin.orientation.z = 0
occupancy_grid.info.origin.orientation.w = 1

rospy.Subscriber("scan", LaserScan, scanCallback, queue_size=100)
pub_grid = rospy.Publisher("scan_grid", OccupancyGrid, queue_size=100)

rospy.spin()
