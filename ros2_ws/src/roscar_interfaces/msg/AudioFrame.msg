# One 40ms frame from the sole capture owner: 16kHz mono PCM16 LE.
# stamp is frame publication time; sequence gaps invalidate an utterance.
builtin_interfaces/Time stamp
uint64 sequence
uint32 sample_rate
uint8 channels
uint8[] pcm_s16le
