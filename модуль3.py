fr1 = 8
amplitude = 32
framestep = 1
order = []
pos = int(amplitude/2)
for _ in range(fr1):
    pos -= int(amplitude/fr1)
    order.append(pos)
print(order)
for iii in range(0, fr1, framestep):
    print(order[iii + framestep - 1])
for iii in range(0, fr1, framestep):
    print(- order[iii + framestep - 1])

# [12, 8, 4, 0, -4, -8, -12, -16, -12, -8, -4, 0, 4, 8, 12, 16]