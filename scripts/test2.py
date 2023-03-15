

in_min = 1
in_max = -1
out_max = 550
out_min = 0
jaw_position = 1
jaw_position_mapped = (jaw_position - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
print(jaw_position_mapped)