motion = [0 for _ in range(22)]
jump_motion = [motion.copy() for _ in range(4)]
jump_motion[0][0] = 10
jump_motion[1][0] = jump_motion[2][0] = jump_motion[3][0] = 3
jump_motion[1][1] = -700


print(jump_motion)