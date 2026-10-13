# Selection lifecycle only; never grants motion authority.
std_msgs/Header header
# UNCLAIMED / TRACKING / LOST
string state
string target_id
string person_id
# Increases for each transition in this tracker process.
uint64 sequence
# ready / claimed / lost / returned / expired / released
string event
float32 lost_for_s
float32 reclaim_after_s
bool can_claim
