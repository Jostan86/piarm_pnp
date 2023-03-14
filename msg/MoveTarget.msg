# x, y, z location is the location of the gripper end relative to the surface beneath the center
# of the base of the arm. z is height, x and y are perpendiclar and planer with the surface, y is straight forward and
# backward, with positive being forward, x is right and left, with negative left (from arm perspective).
# Pitch is the angle of the gripper relative to the surface, at angle 0 it sticks straight up, at -90 it flat, at -180
# is faces down. It also has an upper an lower bound of allowable pitch in case the given pitch is unreachable. To my
# understanding, it will only use the non-target value if it has to, I'm not sure how it decides what to use
float64 target_x            # x location to move arm to
float64 target_y            # y location to move arm to
float64 target_z            # z location to move arm to
int32 target_pitch          # pitch of gripper to move arm to
int32 pitch_lower_limit     # lower limit of allowable pitch, should be a smaller (or more negative) number than pitch
int32 pitch_upper_limit     # upper limit of allowable pitch