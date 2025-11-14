uint8 ZYX = 0	    # ordinary setting
uint8 ZXY = 1			# to avoid gimbal lock of pitch 90
uint64 timestamp		# time since system start (microseconds)

uint8 order
float32 roll
float32 pitch
float32 yaw
float32[3] eulers
