######################################################
## Specifies a Region of Interest (ROI) in an image ##
######################################################

# camera enums 
uint8 FRONT=0
uint8 BOTTOM=1

# Information about the image origin
uint8 camera

# Identification information about the ROI
string[] tags
float32 confidence
float32 actualHeight
float32 actualWidth

# Pixel locations in image as a complex polygon
geometry_msgs/Point[] polygon

# Bounding box for the polygon
geometry_msgs/Point   topLeft   # Aligned to the image
uint32 width                    # Bounding box width in pixels
uint32 height                   # Bounding box height in pixels

# Estimated position in real coordinates relative to the robot
# Displacement is represented in meters
# hasOrientation Indicates if there an orientation is specified
# in the pose
geometry_msgs/Pose poseEstimate
uint8 hasOrientation
