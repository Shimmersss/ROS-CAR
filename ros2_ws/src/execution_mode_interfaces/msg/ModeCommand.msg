# ENTER switches/starts a mode; RETAKE and REPEAT apply to the active session;
# EXIT cancels the entire mode. A manager forwards controls with its session_id.
std_msgs/Header header
string request_id
string session_id
string mode_id
string operation
