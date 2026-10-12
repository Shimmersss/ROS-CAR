# ByteTrack epoch:track_id; empty for an absent retained identity.
string track_id
# Session-local identity, NOT a biometric identification or a control target.
# Empty unless appearance association has been confirmed.
string person_id
bool verified
bool visible
# PENDING/REGISTERED/VERIFIED/REASSOCIATED/AMBIGUOUS/UNVERIFIED/LOST/UNCONFIRMED
string state
# Cosine distance, not a probability; NaN when unavailable.
float32 appearance_distance
float32 last_seen_age_s
string detail
