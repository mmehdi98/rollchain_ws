# Rod tensions at the base [N].
#
# With names: each entry is matched to a rod by name (rods.name in the robot
# config), and rods that are not listed keep their previous tension.
# Without names: one tension per rod, in the order of the robot config.
# Negative tensions are treated as 0 (rods cannot push).

std_msgs/Header header
string[] name
float64[] tension
