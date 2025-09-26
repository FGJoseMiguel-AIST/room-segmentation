# ipa_room_segmentation_tools

Cliente ROS1 (Noetic) para el action `ipa_building_msgs/MapSegmentationAction`.

## Uso
```bash
# en una terminal con ROS master y el server de room_segmentation corriendo:
roslaunch ipa_room_segmentation_tools map_to_room_seg_client.launch \
  map_topic:=/map \
  server_name:=/room_segmentation/room_segmentation_server

Dependencias

ROS Noetic

ipa_coverage_planning (para mensajes ipa_building_msgs)

cv_bridge, geometry_msgs, nav_msgs, sensor_msgs, actionlib
