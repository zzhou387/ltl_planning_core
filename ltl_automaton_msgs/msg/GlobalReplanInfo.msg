# Message used to communicate between
# global and local planner

Header header

# Replanning flag
int8 level

# Array of LTL states (also called product states).
LTLState[] ltl_states

# Update info
UpdateInfo update_info