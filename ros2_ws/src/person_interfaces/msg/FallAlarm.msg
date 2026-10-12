# Per-frame fall alarm: the highest fall_stage over person_states and person_states_continued
# of the same RGB observation (same header). Consumers alert on rising stage edges; a fall
# handed from a continued (undetected) person back to a returning detection keeps its stage.
uint8 NONE=0
uint8 SUSPECTED=1
uint8 CONFIRMED=2
std_msgs/Header header
bool valid
uint8 stage                 # highest fall stage this frame
string track_id             # epoch:track_id of that person, '' when stage is NONE
bool continued              # the person has no detection this frame (person_states_continued)
float32[4] box              # that person's box (last detected box when continued), NaN when NONE
bool geometry_confirmed     # pose3d gravity and ground confirmed; otherwise stage stays <= SUSPECTED
uint32 suspected_count      # people at SUSPECTED this frame (detected + continued)
uint32 confirmed_count      # people at CONFIRMED this frame
string detail
