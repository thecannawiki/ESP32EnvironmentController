import json
from datetime import datetime

import matplotlib
import numpy as np
from matplotlib import pyplot as plt
from matplotlib.dates import DateFormatter
import matplotlib.ticker as plticker
from tqdm import tqdm

def drawGraph(times, datalist, title, color, filename=None, base = 10.0):

    fig, ax = plt.subplots(figsize=(26, 16))

    Dates = matplotlib.dates.date2num(times)

    # Set plot title
    ax.set(title=title)

    # Plot graph
    ax.plot(Dates, datalist, linewidth=1, color=color)

    loc = plticker.MultipleLocator(base=base)  # this locator puts ticks at regular intervals
    ax.yaxis.set_major_locator(loc)
    dateLoc = matplotlib.dates.AutoDateLocator()
    ax.xaxis.set_major_locator(dateLoc)

    # Set gridlines
    ax.grid(True, linestyle='--')

    date_form = DateFormatter("%m-%d %H:%M")
    ax.xaxis.set_major_formatter(date_form)
    #plt.show()

    if filename is not None:
        fig.savefig(filename+".png", dpi=fig.dpi, bbox_inches='tight')
    else:
        plt.show()




try:
    times = []
    co2 = []
    humidity = []
    temp = []

    file1 = open("environ.csv", 'r')
    Lines = file1.readlines()

    # Strips the newline character
    for line in tqdm(Lines):
        print(line)
        info = json.loads(line)
        if len(info) == 5:
            times.append(datetime.fromtimestamp(info["time"] / 1000.0))
            co2num = info["co2"]
            if co2num > 5000:
                co2num = 5000
            co2.append(co2num)
            temp.append(info["temp"])
            humidity.append(info["humidity"])

    co2800count = 0
    co2500count = 0
    for i in co2:
        if i >= 800:
            co2800count+=1
        if i >=500:
            co2500count+=1

    print("co2 > 800ppm ", str(co2800count/len(co2) * 100) + "%", "of the time")
    print("co2 > 500ppm ", str(co2500count/len(co2) * 100) + "%", "of the time")
    print("")
    drawGraph(times, co2, "co2 (ppm)", "blue", "co2chart", 100.0)
    drawGraph(times, temp, "Temperature (C)", "blue", "tempchart", 100.0)
    drawGraph(times, humidity, "Humidity (RH %)", "blue", "humiditychart", 100.0)

except Exception as e:
    print(e)



