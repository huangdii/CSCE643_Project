#!/usr/bin/env python
import sys
print (sys.path)
import numpy as np
print (np.__file__)
print (np.__version__)
import os
import gym

import matplotlib
import matplotlib.pyplot as plt
import itertools
import sys
import argparse

from scipy.interpolate import pchip
from gym.wrappers.monitor import load_results

class LivePlot(object):
    def __init__(self, outdir, data_key='episode_lengths', line_color='blue'):
        self.outdir = outdir
        self._last_data = None
        self.data_key = data_key
        self.line_color = line_color

        matplotlib.rcParams['toolbar']='None'
        plt.style.use('ggplot')
        plt.xlabel("episodes")
        plt.ylabel("length of episode")
        fig = plt.gcf().canvas.set_window_title('simulation_graph')
        matplotlib.rcParams.update({'font.size': 15})

    def plot(self, full=True, dots=False, average=0, interpolated=0):
        results = load_results(self.outdir)
        data = results[self.data_key]
        avg_data = []

        if full:
            plt.plot(data, color='blue')
        if dots:
            plt.plot(data, '.', colot='black')
        if average > 0:
            average = int(average)
            for i, val in enumerate(data):
                if i%average==0:
                    if (i+average) < len(data)+average:
                        avg = sum(data[i:i+average])/average
                        avg_data.append(avg)
            new_data = expand(avg_data, average)
            plt.plot(new_data, color='red', linewidth=2.5)
        if interpolated > 0:
            avg_data= []
            avg_data_points = []
            n = len(data)/interpolated
            if n == 0:
                n = 1
            data_fix = 0
            for i, val in enumerate(data):
                if i%n == 0:
                    if (i+n) <= len(data)+n:
                        avg = sum(data[i:i+n])/n
                        avg_data.append(avg)
                        avg_data_points.append(i)
                if (i+n) == len(data):
                    data_fix = n
            
            x = np.arange(len(avg_data))
            y = np.array(avg_data)
            interp = pchip(avg_data_points, avg_data)
            xx = np.linspace(0, len(data)-data_fix, 1000)
            plt.plot(xx, interp(xx), color='green', linewidth=3.5)

def expand(lst, n):
    lst = [[i]*n for i in lst]
    lst = list(itertools.chain.from_iterable(lst))
    return lst

def pause():
    programPause = raw_input("Press the <ENTER> key to finish...")

if __name__ == '__main__':
    outdir = '/home/jiyoon/python3_ws/src/MobileRobotRL/rl_rapid/rapid_results'
    plotter = LivePlot(outdir)

    parser = argparse.ArgumentParser()
    parser.add_argument("-f", "--full", action='store_true', help="print the full data plot with lines")
    parser.add_argument("-d", "--dots", action='store_true', help="print the full data plot with dots")
    parser.add_argument("-a", "--average", type=int, nargs='?', const=50, metavar="N", help="plot an averaged graph")
    parser.add_argument("-i", "--interpolated", type=int, nargs='?', const=50, metavar="M", help="plot an interpolated graph")
    
    if len(sys.argv)==1:
        plotter.plot(full=True)
    else:
        plotter.plot(full=args.full, dots=args.dot, average=args.average, interpolated=args.interpolated)

    plt_save_path = "/home/jiyoon/python3_ws/src/MobileRobotRL/rl_rapid/rapid_results/results_steps_plot.png"
    plt.savefig(plt_save_path)
    print ("Saved plot in "+plt_save_path)
    print ("Showing Plot in RemoteScreen")
    plt.show()
