#!/usr/bin/env python
from typing import List
from geometry_msgs.msg import Vector3
from occupancy_field import OccupancyField
from constants import PROB_THRESHOLD
#update
#calculate new prob based on original pos and new relative pos prob
# add new locs to storage (with probs) and remove old locs
# might not need this as a node, just ability to hold and call functions

# class UpdateLocationsNode(object):
#     def __init__(self):
#         # rospy.init_node('update_location_node')

#     def 


class Locations():
    def __init__(self):
        self.occupancy_field = OccupancyField()
        self.locations = {} # location to be stored as tuple(x, y, theta): confidence
        self.confidence_func = lambda new_confidence, old_confidence: (new_confidence + old_confidence) / 2

    def vector_to_tuple(v):
        return (v.x, v.y, v.z)

    def update_locations(potential_locations: List):
        # just assuming the structure of potential_locations to be a list of tuples (cur_loc(vector3), prev_loc, confidence)
        # going to average old confidence with new confidence
        new_locations = {}
        for cur_loc, prev_loc, confidence in potential_locations:
            # new_conf = (confidence + self.locations[self.vector_to_tuple(prev_loc)]) / 2
            new_conf = self.confidence_func(confidence, self.locations[self.vector_to_tuple(prev_loc)])
            if new_conf >= PROB_THRESHOLD:
                new_locations[self.vector_to_tuple(cur_loc)] = new_conf
        self.locations = new_locations

    def get_locations():
        return self.locations
    


