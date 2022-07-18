#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import sys
from math import sqrt, dist
import json

def read(name):
    f = open(f"/home/cas/pmos/enchilada/kernel/drivers/gpu/drm/panel/{name}.h", "r")
    lines = f.readlines()
    f.close()

    data = {}
    key = ""
    i = 0

    for l in lines:
        if l.startswith("// "):
            key = l.split(" ")[-1][:-1]
            if key in data:
                print(f"{key} already defined!")
                exit(1)
            print(key)
            data[key] = [[], [], []]
            i = 0
            continue
        elif l.startswith("///"):
            continue
        # e.g.: "{ 0x8D, 0xD0 },"
        leftcol = int(l.split(",")[0].split(" ")[1], 16)
        rightcol = int(l.split(",")[1][1:].split(" ")[0], 16)
        data[key][0].append(leftcol)
        data[key][1].append(rightcol)
        #print(hex(rightcol))
        data[key][2].append(i)

        i+=1
    
    return data

pkey = ""
pv = 0
pressed = False

rgb = {}

fig, ax = plt.subplots()

def update():
    global rgb
    val = rgb[pkey]
    rgb[pkey][-1].remove()
    rgb[pkey][-1] = ax.scatter(range(len(val[0])), val[1], label=pkey, color=pkey)
    #for key, val in rgb.items():
        # vals = []
        # for x in range(0, int(len(val[1])), 2):
        #     vals.append((val[1][x] << 8 | val[1][x+1]))
        #rgb[key][-1].set_offsets([range(len(val[0])), val[1]])


def onmove(event):
    if not pressed:
        return
    print("%d, %d" % (event.xdata, event.ydata))
    rgb[pkey][1][pv] = round(event.ydata)
    update()


def mousepress(event):
    global pressed, rgb, pkey, pv
    pressed = True
    x, y = round(event.xdata), round(event.ydata)
    cx = 99999
    cy = 99999
    for key, val in rgb.items():
        for i in range(len(val[0])):
            vy = val[1][i]
            d1, d2 = dist([x, y], [i, vy]), dist([x, y], [cx, cy])
            #print(f"d1: {round(d1)}, d2: {round(d2)}")
            if d1 < d2:
                pkey = key
                pv = i
                cx, cy = i, vy

    print("closest: %s (%d): (m: %d, %d), (c: %d, %d)" % (pkey, pv, x, y, cx, cy))


def mouserelease(event):
    global pressed
    pressed = False


def plot():
    global fig, ax
    plt.ion()

    fig.canvas.mpl_connect('motion_notify_event', onmove)
    fig.canvas.mpl_connect('button_press_event', mousepress)
    fig.canvas.mpl_connect('button_release_event', mouserelease)

    ax.plot([0, 30], [0, 1500], color="black")

    for key, val in rgb.items():
        # print(key)
        # for i in range(len(val[0])):
        #     print(f"  {hex(val[1][i])}")
        rgb[key].append(ax.scatter(range(len(val[0])), val[1], label=key, color=key))

    #print(colors)
    plt.xlabel("Column A")
    plt.ylabel("Column B")

    plt.legend()

    # rect = patches.Rectangle((600, -35), 350, 155, linewidth=1, edgecolor='r', facecolor='none')
    # ax.add_patch(rect)
    plt.show(block=True)
    #plt.draw()
    #plt.waitforbuttonpress()


def write_linear(data):
    f = open("/home/cas/pmos/enchilada/kernel/drivers/gpu/drm/panel/shift6mq_out.h", "w")
    for key, val in data.items():
        f.write(f"// {key}\n")
        for i in range(len(val[0])):
            #if "0_config_arr" in key:
            # v = (int(i/2 + 1) * 50 & 0xff) if i & 0b1 else (int(i/2 + 1) * 50 >> 8)
            # f.write(f"{{ {val[0][i]:#0{4}x}, {v:#0{4}x} }},\n")
            #else:
            f.write(f"{{ {val[0][i]:#0{4}x}, {val[1][i]:#0{4}x} }},\n")

    f.close()

def readcal(cal, i, col):
    v = round(cal[int(i/2)][col] * 1500)
    if i & 0b1:
        v &= 0xff
    else:
        v = v >> 8
    
    return v

def write_calib(data):
    with open("/home/cas/pmos/enchilada/kernel/drivers/gpu/drm/panel/shift6mq_cal.json", "r") as f:
        cal = json.load(f)

    f = open("/home/cas/pmos/enchilada/kernel/drivers/gpu/drm/panel/shift6mq_out.h", "w")
    for key, val in data.items():
        f.write(f"// {key}\n")
        for i in range(len(val[0])):
            if key == "r0_config_arr":
                f.write(f"{{ {val[0][i]:#0{4}x}, {readcal(cal, i, 0):#0{4}x} }},\n")
            elif key == "g0_config_arr":
                f.write(f"{{ {val[0][i]:#0{4}x}, {readcal(cal, i, 1):#0{4}x} }},\n")
            elif key == "b0_config_arr":
                f.write(f"{{ {val[0][i]:#0{4}x}, {readcal(cal, i, 2):#0{4}x} }},\n")
            else:
                f.write(f"{{ {val[0][i]:#0{4}x}, {val[1][i]:#0{4}x} }},\n")

    f.close()

name = ""

if "-l" in sys.argv:
    print("Adjusting current config")
    name = "shift6mq_out"
else:
    name = "shift6mq"

data = read(name)

if "-p" in sys.argv:
    rgb = {"red"   : data["r0_config_arr"],
           "green" : data["g0_config_arr"],
           "blue"  : data["b0_config_arr"]}

    for key, val in rgb.items():
        vals = [[], []]
        for x in range(0, int(len(val[0])), 2):
            vals[0].append((val[0][x] << 8 | val[0][x+1]))
            vals[1].append((val[1][x] << 8 | val[1][x+1]))
        rgb[key][0] = vals[0]
        rgb[key][1] = vals[1]

    plot()

    for key, val in rgb.items():
        vals = [[], []]
        for x in range(0, int(len(val[0]))):
            vals[0].append((val[0][x] >> 8))
            vals[0].append((val[0][x] & 0xff))
            vals[1].append((val[1][x] >> 8))
            vals[1].append((val[1][x] & 0xff))
        rgb[key][0] = vals[0]
        rgb[key][1] = vals[1]

    data["r0_config_arr"] = rgb["red"]
    data["g0_config_arr"] = rgb["green"]
    data["b0_config_arr"] = rgb["blue"]
    print("Plotted!")

print(data)

if "-c" in sys.argv:
    write_calib(data)
else:
    write_linear(data)
