# Robot Localization Node (Sensor Fusion only, not localization)

This node takes inputs from multiple odometry sources like visual odometry, LIDAR odometry, etc

The output is /odometry/filtered and the publishing frequency is defined in the config file.

# TF2 Note

Since this frame will be publishing the odom->base_footprint transform, ensure the other
odometry nodes do not publish the same.

If the same tf transform is being published by other nodes, something may get messed up.