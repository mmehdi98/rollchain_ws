# State of the robot after a solve. Rod arrays are in the order of the robot
# config (rods.name).
#
# When converged is false, only header, name, tension_base, converged and
# residual are filled; joint_states keep the last converged shape.

std_msgs/Header header

string[] name
float64[] tension_base     # tension at the base, the input clamped to >= 0 [N]
float64[] tension_anchor   # tension where the rod is fixed, after friction [N]
float64[] pull_in          # geometric pull-in at the base [m], + = drawn toward the actuator (theory RC19)

bool converged
float64 residual           # largest joint-balance residual [N m]
bool stable                # false: the equilibrium is unstable, the real robot would buckle away from it
float64 stability_margin   # smallest stiffness eigenvalue [N m/rad]; < 0 = unstable
float64 max_strain         # peak rod fibre strain [-]
uint32[] joints_at_stop    # joints outside their range, numbered from 1
float64 min_normal_force   # smallest link-link contact normal force [N]
float64 max_slip_ratio     # largest tangential / normal contact force
