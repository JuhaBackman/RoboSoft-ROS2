# Associated observations. Measurements use measurement_frame_id, map
# positions use map_frame_id and map_id identifies the active TASK snapshot.
std_msgs/Header header
string measurement_frame_id
string map_frame_id
string map_id
AssociatedLandmark[] landmarks
