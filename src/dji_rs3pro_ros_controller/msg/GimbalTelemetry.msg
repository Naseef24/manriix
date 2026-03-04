# Header header
float32[6] angle          # degrees
float32[6] vel            # deg/s
float32[6] current        # amps
# time armReadTimestamp 
builtin_interfaces/Time start_sync_time 
uint32 isr_ticks          # this would overflow if the robot is left on for 497 days straight at 100 hz 
uint8 buffer_health
