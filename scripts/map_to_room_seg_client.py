#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
import numpy as np
from threading import Lock
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ipa_building_msgs.msg import MapSegmentationAction, MapSegmentationGoal, MapSegmentationResult

class MapToRoomSegClient(object):
    def __init__(self):
        # Params
        self.map_topic = rospy.get_param("~map_topic", "/map")
        # IMPORTANTE: el servidor del action corre bajo el ns 'room_segmentation' según tu launch
        self.server_name = rospy.get_param("~server_name", "/room_segmentation/room_segmentation_server")
        self.debounce_sec = rospy.get_param("~debounce_sec", 2.0)
        self.robot_radius = rospy.get_param("~robot_radius", 0.30)  # m (ajusta a tu robot)
        self.return_pixel = rospy.get_param("~return_format_in_pixel", True)
        self.return_meter = rospy.get_param("~return_format_in_meter", False)
        self.algorithm = rospy.get_param("~room_segmentation_algorithm", 3)  # 1 morph, 2 dist, 3 voronoi, 5 VRF ...

        self._bridge = CvBridge()
        self._last_sent = rospy.Time(0)
        self._client = actionlib.SimpleActionClient(self.server_name, MapSegmentationAction)
        self._goal_lock = Lock()
        self._goal_active = False

        rospy.loginfo("Esperando action server: %s", self.server_name)
        self._client.wait_for_server()
        rospy.loginfo("¡Conectado!")

        rospy.Subscriber(self.map_topic, OccupancyGrid, self._map_cb, queue_size=1)

    def _map_to_mono8(self, grid):
        """
        Convierte OccupancyGrid a imagen mono8:
        - libre (0)   -> 255 (blanco)
        - ocupado (100) y unknown (-1) -> 0 (negro)
        """
        w = grid.info.width
        h = grid.info.height
        data = np.asarray(grid.data, dtype=np.int16).reshape(h, w)

        mono = np.zeros((h, w), dtype=np.uint8)
        mono[data == 0] = 255       # free
        # ocupado 100 y unknown -1 quedan en 0
        return mono

    def _send_goal(self, grid):
        img = self._map_to_mono8(grid)
        img_msg = self._bridge.cv2_to_imgmsg(img, encoding='mono8')

        goal = MapSegmentationGoal()
        goal.input_map = img_msg
        goal.map_resolution = grid.info.resolution
        # ipa_room_segmentation espera un Pose (usa position.x/y como origen del mapa)
        goal.map_origin = Pose()
        goal.map_origin.position.x = grid.info.origin.position.x
        goal.map_origin.position.y = grid.info.origin.position.y
        goal.return_format_in_pixel = self.return_pixel
        goal.return_format_in_meter = self.return_meter
        goal.robot_radius = float(self.robot_radius)
        # Si quieres forzar un algoritmo concreto para ESTA ejecución:
        goal.room_segmentation_algorithm = int(self.algorithm)  # 0 = usar el que esté en dynamic_reconfigure

        with self._goal_lock:
            self._goal_active = True
        self._client.send_goal(goal,
                               done_cb=self._done_cb,
                               active_cb=self._active_cb,
                               feedback_cb=self._feedback_cb)

    def _active_cb(self):
        rospy.loginfo("Segmentation goal activo...")

    def _feedback_cb(self, fb):
        # El action no publica feedback útil; dejamos el hook por si lo añades
        pass

    def _done_cb(self, state, result):
        with self._goal_lock:
            self._goal_active = False
        if state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("¡Segmentación OK! Rooms: %d",
                          len(result.room_information_in_pixel) if self.return_pixel else len(result.room_information_in_meter))
            # Ejemplo: puedes publicar/guardar el mapa segmentado si lo quieres como imagen:
            # seg = self._bridge.imgmsg_to_cv2(result.segmented_map, desired_encoding='32SC1')
            # ... (colorear y publicar)
        else:
            rospy.logwarn("Goal terminó con estado: %d", state)

    def _map_cb(self, grid):
        # Debounce + evita solapar goals
        now = rospy.Time.now()
        if (now - self._last_sent).to_sec() < self.debounce_sec:
            return
        with self._goal_lock:
            if self._goal_active:
                return
        self._last_sent = now
        try:
            self._send_goal(grid)
        except Exception as e:
            rospy.logerr("Fallo enviando goal: %s", str(e))

if __name__ == "__main__":
    rospy.init_node("map_to_room_seg_client")
    node = MapToRoomSegClient()
    rospy.spin()
