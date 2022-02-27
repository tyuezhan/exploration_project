#!/usr/bin/env python3

import rospy
from grid_map_rospy import *
import heapq

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

import visualization_msgs

EXPL_TARGET_SET = 1
EXPL_FINISHED   = 2
EXPL_WAITING    = 3
EXPL_FAILED     = 4

class FrontierPlanner:
    def __init__(self, map_frame):
        self.map_frame_ = map_frame
        self.path_publisher = rospy.Publisher("frontier_search_path", Marker, queue_size=5)

    # return next goal
    # Here as an example, only show the implementation of the nearest frontier
    def findExplorationTarget(self, map_in, start):
        # Create some workspace for the wavefront algorithm
        self.map_in_ = map_in
        mapSize = map_in.getSize()
        resolution = map_in.getResolution()
        parent_dic = {}
        foundFrontier = False
        queue = []
        closed = [-1 for _ in range(mapSize)]

        startPoint = (0.0, start)
        # Initialize the queue with the robot position
        heapq.heappush(queue, startPoint)
        closed[start] = 0

        while len(queue) != 0:
            next = heapq.heappop(queue)
            distance = next[0]
            index = next[1]

            if map_in.isFrontier(index):
                # reached boarder of map.
                foundFrontier = True
                path = self.constructPath(index, parent_dic)
                marker = Marker()
                marker.scale.x = .05
                marker.scale.y = .05
                marker.scale.z = .05
                marker.color.a = 1
                marker.color.r = 1
                marker.pose.orientation.w = 1.0
                marker.header.frame_id = self.map_frame_
                marker.header.stamp = rospy.Time.now()
                marker.type = visualization_msgs.msg.Marker.SPHERE_LIST
                path.append(self.index_to_pos(start))
                for element in path:
                    pt = Point()
                    pt.x = element[0]
                    pt.y = element[1]
                    pt.z = 0
                    marker.points.append(pt)

                self.path_publisher.publish(marker)
                    
                return EXPL_TARGET_SET, index
            else:
                ind = []
                ind.append(index - 1)
                ind.append(index + 1)
                ind.append(index - map_in.getWidth())
                ind.append(index + map_in.getWidth())

                for i in range(4):
                    neighbor_idx = ind[i]
                    if map_in.isFree(neighbor_idx) and closed[neighbor_idx] == -1:
                        parent_dic[neighbor_idx] = index
                        heapq.heappush(queue, (distance+resolution, neighbor_idx))
                        closed[neighbor_idx] = distance + resolution
        return EXPL_FAILED, -1
        rospy.logerr("[Frontier Planner] No frontiers.")

    def constructPath(self, current_pos, parent_dic):
        path_index = [current_pos]
        while current_pos in parent_dic.keys():
            current_pos = parent_dic[current_pos]
            path_index.append(current_pos)
        path = []
        for i in range(len(path_index)):
            path.append(self.index_to_pos(path_index[i]))
        return path

    def index_to_pos(self, ind):
        cell_x, cell_y = self.map_in_.getCoordinates(ind)
        world_x = self.map_in_.getOriginX() + (cell_x + 0.5) * self.map_in_.getResolution()
        world_y = self.map_in_.getOriginY() + (cell_y + 0.5) * self.map_in_.getResolution()
        return (world_x, world_y)