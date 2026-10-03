# Progress reaching 1 means the command finished; state=completed confirms arrival.
std_msgs/Header header
string task_id
string state
string message
bool active
float64 progress
float64 elapsed_time
float64 planned_duration
string[] joint_names
float64[] desired_positions
float64[] actual_positions
float64[] position_errors
