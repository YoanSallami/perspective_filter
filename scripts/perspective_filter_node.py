#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import re
import underworlds
import rospy
import sys
from underworlds import *
from underworlds.helpers.geometry import _get_parent_chain
from perspective_filter.msg import Perspective, PerspectiveArray
from perspective_filter.srv import AddAgent, RemoveAgent
from underworlds.tools.visibility import VisibilityMonitor
from underworlds.types import *

EPSILON = 0.001

class PerspectiveFilter(object):
    def __init__(self, ctx, source_world):
        """
        Initialize the beliefs filter
        :param ctx: The underworlds context
        :param source_world: The source world to monitor
        """

        # The underworlds context
        self.ctx = ctx
        # The source world name to monitor
        self.source_world_name = source_world
        # The source world to monitor
        self.source = self.ctx.worlds[source_world]
        # The beliefs worlds map stored by agent name
        self.beliefs = {}
        # The node mapping between source & beliefs worlds stored by agent name
        self.node_mapping = {}
        # The visible nodes stored by agent ID
        self.visible_nodes = {}

        self.cameras = {}

        self.visibilities_enabled_ids = []

        self.tom_enabled_ids = []

        self.agents = []

        self.visible_nodes = {}

        self.relations_map = {}

        self.visibility_monitor = None

        self.nodes_transform = {}

        # The ROS subscribers
        self.ros_subscribers = {}

        # The ROS publishers
        self.ros_publishers = {
            "perspectives": rospy.Publisher("perspective_filter/agent_perspectives", PerspectiveArray, queue_size=10)}
        # The ROS services
        self.ros_services = {
            "add_agent": rospy.Service("perspective_filter/add_agent", AddAgent, self.handle_add_agent),
            "remove_agent": rospy.Service("perspective_filter/remove_agent", RemoveAgent, self.handle_remove_agent)}

    def get_all_cameras(self):
        cameras = {}
        for node in self.source.scene.nodes:
            if node.type == CAMERA:
                cameras[node.id] = node
        return cameras

    def compute_perspective_taking(self, agent_id):
        return self.visible_nodes[agent_id]

    def handle_add_agent(self, req):
        for node in self.source.scene.nodes:
            if node.name == req.agent_name and node.type == CAMERA:
                self.visibilities_enabled_ids.append(node.id)
                if req.tom_lvl >= 1:
                    self.tom_enabled_ids.append(node.id)
                return True
        return False

    def handle_remove_agent(self, req):
        for node in self.source.scene.nodes:
            if node.name == req.agent_name and node.type == CAMERA:
                self.visibilities_enabled_ids.remove(node.id)
                self.tom_enabled_ids.remove(node.id)
                return True
        return False

    def filter(self):
        """
        The method compute the geometric filtering from source world to the
        beliefs worlds of each agent according to the visibilities
        :return: -
        """
        new_nodes_to_update = {}
        nodes_to_update = {}

        self.cameras = self.get_all_cameras()

        if self.visibilities_enabled_ids:
            for agent_id in self.cameras.keys():
                if agent_id in self.tom_enabled_ids:
                    nodes_to_update[agent_id] = []
                    new_nodes_to_update[agent_id] = []
                    if agent_id not in self.beliefs:
                        self.beliefs[agent_id] = self.ctx.worlds[agent_id]
                        self.node_mapping[agent_id] = {}

            dq = deque()
            dq.append(self.source.scene.rootnode)

            while not rospy.is_shutdown() and 0 < len(dq):
                node = dq.pop()
                if node.id != self.source.scene.rootnode.id:
                    # Process start here

                    if node.id in self.cameras.keys():  # if the node is the agent POV
                        nodes_to_update[node.id].append(node)  # we add it to his belief

                    if node.parent in self.cameras.keys():  # if the node is part of an agent
                        nodes_to_update[node.parent].append(node)  # we add it to his belief

                    for agent_id, visible_nodes in self.visible_nodes.items():  # then we add the visible nodes
                        if node in visible_nodes:
                            nodes_to_update[agent_id].append(node)

                # And end here
                for child_id in node.children:
                    dq.append(self.source.scene.nodes[child_id])

            for agent_id, nodes in nodes_to_update.items():
                if nodes:
                    for node in nodes:
                        new_node = node.copy()

                        if node.id in self.node_mapping[agent_id]:
                            new_node.id = self.node_mapping[agent_id][node.id]
                            if new_node.id in self.nodes_transform:
                                if not numpy.allclose(self.nodes_transform[new_node.id], new_node.transformation):
                                    new_nodes_to_update[agent_id].append(new_node)
                        else:
                            self.node_mapping[agent_id][node.id] = new_node.id
                            new_nodes_to_update[agent_id].append(new_node)

            # Finally we update the corresponding beliefs worlds
            for agent_id, nodes in new_nodes_to_update.items():
                for node in nodes:
                    node.parent = self.node_mapping[agent_id][node.parent] if node.parent in self.node_mapping[agent_id] \
                        else self.beliefs[agent_id].scene.rootnode.id
                    self.nodes_transform[new_node.id] = new_node.transformation
                if nodes:
                    rospy.logwarn(nodes)
                    self.beliefs[agent_id].scene.nodes.update(nodes)

    def start_see_situation(self, world, subject, object):
        world.timeline.start(Situation(desc="see("+subject.name+","+object.name))

    def start_visible_situation(self, world, subject, object):
        world.timeline.start(Situation(desc="visible("+subject.name+","+object.name))

    def end_situation(self, world, situation_id):
        world.timeline.end(world.timeline.situations[situation_id])

    def publish_perspectives(self):

        objects_seen = {}
        agents_seen = {}
        for agent_name, visible_nodes in self.visible_nodes.items():
            objects_seen[agent_name] = []
            agents_seen[agent_name] = []
            for node in visible_nodes:
                objects_seen[agent_name].append(node.name)

        perspectives = PerspectiveArray()
        for key, objects in objects_seen.items():
            perspective = Perspective()
            perspective.agent_id = key
            perspective.objects_seen = objects
            perspective.agents_seen = agents_seen[key]
            perspectives.perspectives.append(perspective)

        self.ros_publishers["perspectives"].publish(perspectives)

    def compute_relations(self):
        """
        Compute the visible relations and manage the corresponding situations in the source world
        :param previous_relations_map: The previous relations map
        :return: The current relations map
        """

        previously_visible_nodes = {}
        visible_nodes = {}
        for camera_name, camera_id in self.cameras.items():
            try:
                if self.visible_nodes:
                    previously_visible_nodes[camera_id] = self.visible_nodes[camera_id]
                if self.visibility_monitor is None:
                    self.visibility_monitor = VisibilityMonitor(self.ctx, self.source)
                    rospy.loginfo("[perspective_filter] Visibility monitor now running...")
                    rospy.loginfo(
                        "[perspective_filter] Please activate the Pygame window to enable visibilities computation !")

                if camera_id in self.visibilities_enabled_ids:
                    visible_nodes[camera_id] = self.visibility_monitor.from_camera(camera_name)
            except Exception:
                pass
        self.visible_nodes = visible_nodes

        for agent_id, nodes_seen in self.visible_nodes.items():
            agent = self.source.scene.nodes[agent_id]
            for node in nodes_seen:
                if agent_id in previously_visible_nodes:
                    if node not in previously_visible_nodes[agent_id]:
                        sit_id = self.start_see_situation(self.source, agent, node).id
                        self.relations_map["see("+agent.id+","+node.id+")"] = sit_id
                        sit_id = self.start_visible_situation(self.source, node, agent).id
                        self.relations_map["visible("+node.id+","+agent.id+")"] = sit_id

        for agent_id, nodes_previously_seen in previously_visible_nodes.items():
            agent = self.source.scene.nodes[agent_id]
            for node in nodes_previously_seen:
                if node not in self.visible_nodes[agent_id]:
                    sit_id = self.relations_map["see("+agent.id+","+node.id+")"]
                    self.end_situation(self.source, sit_id)
                    sit_id = self.relations_map["visible("+node.id+","+agent.id+")"]
                    self.end_situation(self.source, sit_id)

    def run(self):
        """
        Run the perspective filter
        :return:
        """
        while not rospy.is_shutdown():
            try:
                self.compute_relations()
                self.publish_perspectives()
                self.filter()
            except KeyboardInterrupt:
                break


if __name__ == '__main__':
    sys.argv = [arg for arg in sys.argv if "__name" not in arg and "__log" not in arg]
    sys.argc = len(sys.argv)
    import argparse

    parser = argparse.ArgumentParser(description="Monitor visibilities & manage the beliefs worlds")
    parser.add_argument("source_world", help="Underworlds source world to monitor")
    args = parser.parse_args()

    rospy.init_node("perspective_filter", anonymous=True)
    with underworlds.Context("Perspective Filter") as ctx:
        PerspectiveFilter(ctx, args.source_world).run()
