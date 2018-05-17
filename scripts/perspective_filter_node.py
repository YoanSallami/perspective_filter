#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import re
import underworlds
import rospy
import sys
from underworlds import *
from underworlds.helpers.transformations import *
from underworlds.helpers.geometry import *
from std_msgs.msg import String, Header
from geometry_msgs.msg import Pose, PoseStamped
from perspectives_msgs.msg import Perspective, PerspectiveArray, PerspectiveArrayStamped, Agent, Object
from perspective_filter.srv import AddAgent, RemoveAgent
from underworlds.tools.visibility import VisibilityMonitor
from underworlds.types import *

EPSILON = 0.025

class PerspectiveFilter(object):
    def __init__(self, ctx, source_world, reference_frame):
        """
        Initialize the beliefs filter
        :param ctx: The underworlds context
        :param source_world: The source world to monitor
        """

        # The underworlds context
        self.ctx = ctx
        # The source world name to monitor
        self.source_world_name = source_world

        self.reference_frame = reference_frame
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
        self.previously_visible_nodes = {}

        self.current_situations_map = {}

        self.visibility_monitor = None

        self.nodes_transform = {}

        # The ROS subscribers
        self.ros_subscribers = {}

        # The ROS publishers
        self.ros_publishers = {
            "perspectives": rospy.Publisher("perspective_filter/agent_perspectives", PerspectiveArrayStamped, queue_size=10)}
        # The ROS services
        self.ros_services = {
            "add_agent": rospy.Service("perspective_filter/add_agent", AddAgent, self.handle_add_agent),
            "remove_agent": rospy.Service("perspective_filter/remove_agent", RemoveAgent, self.handle_remove_agent)}

        self.log_pub = {"isVisibleBy": rospy.Publisher("predicates_log/visibleby", String, queue_size=5)}

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

        for agent_id in self.cameras.keys():
            nodes_to_update[agent_id] = []
            new_nodes_to_update[agent_id] = []
            if agent_id not in self.beliefs:
                world_name = self.cameras[agent_id].name.replace("-","_")+"_beliefs"
                rospy.logdebug("[perspective_filter] create new world <%s>" % str(world_name))
                self.beliefs[agent_id] = self.ctx.worlds[world_name]
                self.node_mapping[agent_id] = {}

        dq = deque()
        dq.append(self.source.scene.rootnode)

        while not rospy.is_shutdown() and 0 < len(dq):
            node = dq.pop()
            if node.id != self.source.scene.rootnode.id:
                # Process start here
                if node.id in self.cameras.keys():  # if the node is the agent POV
                    nodes_to_update[node.id].append(node)  # we add it to his belief

                if node.parent in self.cameras.keys() and node.type == MESH:  # if the node is part of an agent
                    nodes_to_update[node.parent].append(node)  # we add it to his belief

                for agent_id, visible_nodes in self.visible_nodes.items():  # then we add the visible nodes
                    if agent_id in self.cameras.keys():
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
                                self.nodes_transform[new_node.id] = new_node.transformation
                        else:
                            self.nodes_transform[new_node.id] = new_node.transformation
                            new_nodes_to_update[agent_id].append(new_node)
                    else:
                        self.node_mapping[agent_id][node.id] = new_node.id
                        new_nodes_to_update[agent_id].append(new_node)

        # Finally we update the corresponding beliefs worlds
        for agent_id, nodes in new_nodes_to_update.items():
            for node in nodes:
                node.parent = self.node_mapping[agent_id][node.parent] if node.parent in self.node_mapping[agent_id] \
                    else self.beliefs[agent_id].scene.rootnode.id
            if nodes:
                self.beliefs[agent_id].scene.nodes.update(nodes)

    def start_predicate(self, timeline, predicate, subject_name, object_name=None, isevent=False):
        if object_name is None:
            description = predicate + "(" + subject_name + ")"
        else:
            description = predicate + "(" + subject_name + "," + object_name + ")"
        sit = Situation(desc=description)
        sit.starttime = time.time()
        if isevent:
            sit.endtime = sit.starttime
        self.current_situations_map[description] = sit
        self.log_pub[predicate].publish("START " + description)
        timeline.update(sit)
        return sit.id

    def end_predicate(self, timeline, predicate, subject_name, object_name=None):
        if object_name is None:
            description = predicate + "(" + subject_name + ")"
        else:
            description = predicate + "(" + subject_name + "," + object_name + ")"
        try:
            sit = self.current_situations_map[description]
            self.log_pub[predicate].publish("END " + description)
            timeline.end(sit)
        except Exception as e:
            rospy.logwarn("[perspective_filter] Exception occurred : " + str(e))

    def isobject(self, scene, node):
        isobject = False
        if node != scene.rootnode:
            if node.type == MESH:
                if scene.nodes[node.parent].type != CAMERA:
                    isobject = True
        return isobject

    def publish_perspectives(self):
        header = Header()
        header.frame_id = self.reference_frame
        header.stamp = rospy.Time()
        perspective_array_stamped = PerspectiveArrayStamped()
        perspective_array_stamped.header = header
        cpt = 0
        rospy.logdebug("%s perspectives to send"%str(len(self.visible_nodes)))
        for agent_id, visible_nodes in self.visible_nodes.items():
            perspective_array_stamped.perspectives.append(Perspective())
            perspective_array_stamped.perspectives[cpt].agent_id = str(agent_id)
            perspective_array_stamped.perspectives[cpt].agent_name = str(self.source.scene.nodes[agent_id].name)
            for node in visible_nodes:
                if self.isobject(self.source.scene, node):
                    obj_msg = Object()
                    obj_msg.id = node.id
                    obj_msg.name = node.name
                    t = translation_from_matrix(get_world_transform(self.source.scene, node))
                    q = quaternion_from_matrix(get_world_transform(self.source.scene, node))
                    pose = Pose()
                    pose.position.x = t[0]
                    pose.position.y = t[1]
                    pose.position.z = t[2]
                    pose.orientation.x = q[0]
                    pose.orientation.y = q[1]
                    pose.orientation.z = q[2]
                    pose.orientation.w = q[3]
                    pose_stamped = PoseStamped(header, pose)
                    obj_msg.pose = pose_stamped
                    perspective_array_stamped.perspectives[cpt].objects_seen.append(obj_msg)

                if node.type == CAMERA:
                    agent_msg = Agent()
                    agent_msg.id = node.id
                    agent_msg.name = node.name
                    t = translation_from_matrix(get_world_transform(self.source.scene, node))
                    q = quaternion_from_matrix(get_world_transform(self.source.scene, node))
                    pose = Pose()
                    pose.position.x = t[0]
                    pose.position.y = t[1]
                    pose.position.z = t[2]
                    pose.orientation.x = q[0]
                    pose.orientation.y = q[1]
                    pose.orientation.z = q[2]
                    pose.orientation.w = q[3]
                    pose_stamped = PoseStamped(header, pose)
                    agent_msg.head_gaze_pose = pose_stamped
                    agent_bodies = []
                    for child_id in node.children:
                        node = self.source.scene.nodes[child_id]
                        object_msg = Object()
                        object_msg.id = node.id
                        object_msg.name = node.name
                        t = translation_from_matrix(get_world_transform(self.source.scene, node))
                        q = quaternion_from_matrix(get_world_transform(self.source.scene, node))
                        pose = Pose()
                        pose.position.x = t[0]
                        pose.position.y = t[1]
                        pose.position.z = t[2]
                        pose.orientation.x = q[0]
                        pose.orientation.y = q[1]
                        pose.orientation.z = q[2]
                        pose.orientation.w = q[3]
                        pose_stamped = PoseStamped(header, pose)
                        object_msg.pose = pose_stamped
                        agent_bodies.append(object_msg)
                    agent_msg.agent_bodies = agent_bodies
                    perspective_array_stamped.perspectives[cpt].agents_seen.append(agent_msg)
            cpt += 1

        self.ros_publishers["perspectives"].publish(perspective_array_stamped)
        rospy.logdebug("[perspective_filter] %s perspectives published, %s should be" % (str(len(perspective_array_stamped.perspectives)), cpt))

    def compute_relations(self):
        """
        Compute the visible relations and manage the corresponding situations in the source world
        :param previous_relations_map: The previous relations map
        :return: The current relations map
        """

        visible_nodes = {}

        self.cameras = self.get_all_cameras()
        rospy.logdebug(self.cameras)

        if self.cameras.items():
            try:
                if self.visibility_monitor is None:
                    self.visibility_monitor = VisibilityMonitor(self.ctx, self.source)
                    rospy.loginfo("[perspective_filter] Visibility monitor now running, please active the Pygame windows.")
                visible_nodes = self.visibility_monitor.compute_all()
                rospy.logdebug("[perspective_filter] %d perspectives computed " % len(visible_nodes))
                #rospy.logdebug(visible_nodes)
            except Exception as e:
                rospy.logwarn("[perspective_filter] Exception occurred while computing relation : %s" % str(e))
        if self.visibility_monitor:
            self.visible_nodes = {} #visible_nodes
            for camera_name, visibles_obj in visible_nodes.items():
                camera_id = self.source.scene.nodebyname(camera_name)[0].id
                self.visible_nodes[camera_id] = visibles_obj
                for node in visibles_obj:
                    if node.parent in self.cameras.keys():
                        if self.source.scene.nodes[node.parent] not in visibles_obj:
                            visibles_obj.append(self.source.scene.nodes[node.parent])

            for agent_id, nodes_seen in self.visible_nodes.items():
                agent = self.source.scene.nodes[agent_id]
                for node in nodes_seen:
                    if agent_id in self.previously_visible_nodes:
                        if node not in self.previously_visible_nodes[agent_id]:
                            self.start_predicate(self.source.timeline, "isVisibleBy", node.name, object_name=agent.name)
                    else:
                        self.start_predicate(self.source.timeline, "isVisibleBy", node.name, object_name=agent.name)

            for agent_id, nodes_previously_seen in self.previously_visible_nodes.items():
                agent = self.source.scene.nodes[agent_id]
                for node in nodes_previously_seen:
                    if agent_id in self.visible_nodes:
                        if node not in self.visible_nodes[agent_id]:
                            self.end_predicate(self.source.timeline, "isVisibleBy", node.name, object_name=agent.name)
                    else:
                        self.end_predicate(self.source.timeline, "isVisibleBy", node.name, object_name=agent.name)

            self.publish_perspectives()
            self.previously_visible_nodes = self.visible_nodes


    def run(self):
        """
        Run the perspective filter
        :return:
        """
        rate = rospy.Rate(30)
        #rospy.loginfo("[perspective_filter] Start running the ros node")
        while not rospy.is_shutdown():
            try:
                self.compute_relations()
                if self.visibility_monitor:
                    self.filter()
                    #self.publish_perspectives()
                rate.sleep()
            except KeyboardInterrupt:
                break


if __name__ == '__main__':
    sys.argv = [arg for arg in sys.argv if "__name" not in arg and "__log" not in arg]
    sys.argc = len(sys.argv)
    import argparse

    parser = argparse.ArgumentParser(description="Monitor visibilities & manage the beliefs worlds")
    parser.add_argument("source_world", help="Underworlds source world to monitor")
    parser.add_argument("--reference_frame", default="map", help="The reference frame of the system")
    args = parser.parse_args()
    rospy.loginfo("[perspective_filter] Initialize ros node")
    rospy.init_node("perspective_filter", anonymous=True)#, log_level=rospy.DEBUG)
    rospy.loginfo("[perspective_filter] Connecting to Underworlds server...")
    with underworlds.Context("Perspective Filter") as ctx:
        PerspectiveFilter(ctx, args.source_world, args.reference_frame).run()
