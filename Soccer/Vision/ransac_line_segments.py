"""
Module is designed by A.Babaev from MIPT Starkit team
Finding lines by RANSAC algorythm in B/W binary image
usage:

build_line_segments( data, data_size, rank_threshold = 30, line_num_limit = 5)

data - uint8 bytearray with shape (n,2), packed by x,y pairs of white pixels
data_size  < n - number of usefull data
rank_threshold  - detected lines with pixel number less than this number will be ignored
line_num_limit  - limit of detected number of lines

"""
import sys, os

current_work_directory = os.getcwd()
current_work_directory = current_work_directory.replace('\\', '/')
if sys.version != '3.4.0': 
    current_work_directory += '/'
    import numpy as np
else:
    import pyb, sensor
    import ulab as np


import math

#from urandom import getrandbits


#def randrange( start, stop=None):
##helper function for working with random bit sequence
#    if stop is None:
#        stop = start
#        start = 0
#    upper = stop - start
#    bits = 0
#    pwr2 = 1
#    while upper > pwr2:
#        pwr2 <<= 1
#        bits += 1
#    while True:
#        r = getrandbits(bits)
#        if r < upper:
#            break
#    return r + start

#def random():
#    #getting a random number from 0 to 1
#    return randrange(10000) / 10000

#@micropython.native
#def ransac_calc(data1, deviation, number_of_iterations, data_size):
#    best_score = 0
#    best_sample_1 = 0
#    best_sample_2 = 0
#    probe = 0
#    while(probe < number_of_iterations):
#        vertical_line = False
#        y1 = 128
#        while(y1 > 127):
#            sample_1 = int(random()*data_size)
#            y1 = tuple(data1[sample_1])[1]
#        y2 = 128
#        while(y2 > 127):
#            sample_2 = int(random()*data_size)
#            y2 = tuple(data1[sample_2])[1]
#            if sample_1 == sample_2: 
#                y2 = 128
#        x1 = tuple(data1[sample_1])[0]
#        x2 = tuple(data1[sample_2])[0]
#        if x1 == x2: vertical_line = True
#        else:
#            a = float((y1-y2)/(x1-x2))
#            b = float((y2*x1 - y1*x2)/(x1-x2))
#        score = 0
#        for i in range (data_size):
#            if tuple(data1[i])[1] > 127 : continue
#            if i == sample_1 or i == sample_2: continue
#            x = tuple(data1[i])[0]
#            y = tuple(data1[i])[1]
#            if vertical_line: h = x1 - x
#            else:
#                a3 = a * x + b - y
#                if a == 0:
#                    h = a3
#                else:
#                    b3 = x - (y - b) / a
#                    if a3 == 0 and b3 == 0: h = 0
#                    else: h = (a3 * b3)/math.sqrt((a3 * a3 + b3 * b3))
#            if abs(h) < deviation:
#                score += 1
#        if score > best_score:
#            best_sample_1 = sample_1
#            best_sample_2 = sample_2
#            best_score = score
#        probe += 1
#    x1 = tuple(data1[best_sample_1])[0]
#    x2 = tuple(data1[best_sample_2])[0]
#    y1 = tuple(data1[best_sample_1])[1]
#    y2 = tuple(data1[best_sample_2])[1]
#    if x1 == x2: vertical_line = True
#    else:
#        a = float((y1-y2)/(x1-x2))
#        b = float((y2*x1 - y1*x2)/(x1-x2))
#    min_x = 159
#    max_x = 0
#    min_y = 119
#    max_y = 0
#    for i in range (data_size):
#        x = tuple(data1[i])[0]
#        y = tuple(data1[i])[1]
#        if y > 127 : continue
#        if i != best_sample_1 and i != best_sample_2:
#            if vertical_line: h = x1 - x
#            else:
#                a3 = a * x + b - y
#                if a == 0:
#                    h = a3
#                else:
#                    b3 = x - (y - b) / a
#                    if a3 == 0 and b3 == 0: h = 0
#                    else: h = (a3 * b3)/math.sqrt((a3 * a3 + b3 * b3))
#            if abs(h) > deviation: continue
#            if x < min_x : min_x = x
#            if x > max_x : max_x = x
#            if y < min_y : min_y = y
#            if y > max_y : max_y = y
#            data1[i,1] = y + 128
#    if vertical_line: new_segment = [x1, min_y, x1, max_y]
#    elif a == 0: new_segment = [min_x, y1, max_x, y1]
#    else:
#        if (max_x - min_x) > (max_y - min_y):
#            x1 = min_x
#            x2 = max_x
#            y1 = int(a * min_x + b)
#            y2 = int(a * max_x + b)
#        else:
#            y1 = min_y
#            y2 = max_y
#            x1 = int((min_y - b) / a)
#            x2 = int((max_y - b) / a)
#        new_segment = [x1, y1, x2, y2]
#    return new_segment, best_score

#@micropython.native
#def build_line_segments( data, data_size, rank_threshold = 30, line_num_limit = 5):
#    deviation = 2
#    number_of_iterations = 10
#    if data_size < 10: return []
#    rest_number = data_size
#    line_segments_data = []
#    for i in range(line_num_limit):
#        new_segment, inlier_rank= ransac_calc(data, deviation, number_of_iterations, data_size)
#        #print('inlier_rank = ', inlier_rank)
#        if inlier_rank < rank_threshold : break
#        line_segments_data.append(new_segment)
#        rest_number -= inlier_rank
#        if rest_number < 10 : break
#    return line_segments_data

#@micropython.native
def ransac_calc(data1, deviation, number_of_iterations, data_size):
    def random():
        #getting a random number from 0 to 1
        return pyb.rng()/1073741824
    best_score = 0
    best_sample_1 = 0
    best_sample_2 = 0
    probe = 0
    while(probe < number_of_iterations):
        vertical_line = False
        y1 = 128
        while(y1 > 127):
            sample_1 = int(random()*data_size)
            y1 = data1[2*sample_1+1]
        y2 = 128
        while(y2 > 127):
            sample_2 = int(random()*data_size)
            y2 = data1[2*sample_2+1]
            if sample_1 == sample_2:
                y2 = 128
        x1 = data1[2*sample_1]
        x2 = data1[2*sample_2]
        if x1 == x2: vertical_line = True
        else:
            a = float((y1-y2)/(x1-x2))
            b = float((y2*x1 - y1*x2)/(x1-x2))
        score = 0
        for i in range (data_size):
            if data1[2*i+1] > 127 : continue
            if i == sample_1 or i == sample_2: continue
            x = data1[2*i]
            y = data1[2*i+1]
            if vertical_line: h = x1 - x
            else:
                a3 = a * x + b - y
                if a == 0:
                    h = a3
                else:
                    b3 = x - (y - b) / a
                    if a3 == 0 and b3 == 0: h = 0
                    else: h = (a3 * b3)/math.sqrt((a3 * a3 + b3 * b3))
            if abs(h) < deviation:
                score += 1
        if score > best_score:
            best_sample_1 = sample_1
            best_sample_2 = sample_2
            best_score = score
        probe += 1
    x1 = data1[2*best_sample_1]
    x2 = data1[2*best_sample_2]
    y1 = data1[2*best_sample_1+1]
    y2 = data1[2*best_sample_2+1]
    if x1 == x2: vertical_line = True
    else:
        a = float((y1-y2)/(x1-x2))
        b = float((y2*x1 - y1*x2)/(x1-x2))
    min_x = 159
    max_x = 0
    min_y = 119
    max_y = 0
    for i in range (data_size):
        x = data1[2*i]
        y = data1[2*i+1]
        if y > 127 : continue
        if i != best_sample_1 and i != best_sample_2:
            if vertical_line: h = x1 - x
            else:
                a3 = a * x + b - y
                if a == 0:
                    h = a3
                else:
                    b3 = x - (y - b) / a
                    if a3 == 0 and b3 == 0: h = 0
                    else: h = (a3 * b3)/math.sqrt((a3 * a3 + b3 * b3))
            if abs(h) > deviation: continue
            if x < min_x : min_x = x
            if x > max_x : max_x = x
            if y < min_y : min_y = y
            if y > max_y : max_y = y
            data1[2*i+1] = y + 128
    if vertical_line: new_segment = [x1, min_y, x1, max_y]
    elif a == 0: new_segment = [min_x, y1, max_x, y1]
    else:
        if (max_x - min_x) > (max_y - min_y):
            x1 = min_x
            x2 = max_x
            y1 = int(a * min_x + b)
            y2 = int(a * max_x + b)
        else:
            y1 = min_y
            y2 = max_y
            x1 = int((min_y - b) / a)
            x2 = int((max_y - b) / a)
        new_segment = [x1, y1, x2, y2]
    return new_segment, best_score

#@micropython.native
def build_line_segments( img, rank_threshold = 30, line_num_limit = 5, upper_lines = False):
    deviation = 2
    number_of_iterations = 10
    if upper_lines:
        data_size = 160
        data = sensor.alloc_extra_fb(1, 1, 320)
        for x in range(160):
            for y in range(120):
                if img[x + y * 160] > 0: break
            data[2 * x] = x
            data[2 * x + 1] = y
    else:
        data_size = 0
        for i in range(19200):
            data_size += img[i]
        if data_size < 4: return []
        data = sensor.alloc_extra_fb(1, 1, data_size*2)
        data_size = 0
        for x in range(160):
            for y in range(120):
                if img[x + y * 160] > 0:
                    data[2 * data_size] = x
                    data[2 * data_size + 1] = y
                    data_size += 1
    rest_number = data_size
    line_segments_data = []
    for i in range(line_num_limit):
        new_segment, inlier_rank= ransac_calc(data, deviation, number_of_iterations, data_size)
        #print('inlier_rank = ', inlier_rank)
        if inlier_rank < rank_threshold : break
        line_segments_data.append(new_segment)
        rest_number -= inlier_rank
        if rest_number < 2 : break
    sensor.dealloc_extra_fb()
    return line_segments_data