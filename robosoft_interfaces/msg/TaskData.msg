# Complete task data (replaces ISO11783 protobuf)
uint8 STATUS_NOT_SET=0
uint8 STATUS_PLANNED=1
uint8 STATUS_RUNNING=2
uint8 STATUS_PAUSED=3
uint8 STATUS_COMPLETED=4
uint8 STATUS_TEMPLATE=5
uint8 STATUS_CANCELED=6

string task_name          # Task identifier
string farm_name          # Farm/field name
string version            # Task data version
int64 timestamp           # Creation timestamp [unix seconds]

# Route information
GuidanceLine[] guidance_lines       # All guidance lines in task
int32 active_guidance_index         # Currently active guidance line index (-1 if none)

# Zone information
string[] zone_names       # Zone names (e.g., headland, exclusion)
geometry_msgs/Polygon[] zones       # Zone polygons (WGS84 lat/lon)

# Partfield information
string partfield_name     # Field/partfield name
geometry_msgs/Polygon partfield_boundary  # Field boundary (WGS84)

# Persistent lidar landmarks used by AKI localization.
LidarCluster[] lidar_clusters

# Recording information
int32 recorded_point_count  # Number of recorded points

# ISO 11783 TSK.G task status
uint8 status
