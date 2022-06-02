import json
from datetime import datetime

import matplotlib
import numpy as np
from matplotlib import pyplot as plt
from matplotlib.dates import DateFormatter
from tqdm import tqdm

def drawGraph(times, datalist, title, color, filename=None):

    fig, ax = plt.subplots(figsize=(26, 16))

    Dates = matplotlib.dates.date2num(times)
    #orderDates = matplotlib.dates.date2num(orderTimes)

    # Set plot title
    ax.set(title=title)

    plt.xticks(np.arange(min(datalist), max(datalist) + 1, 1.0))
    # Plot graph
    ax.plot(Dates, datalist, linewidth=1, color=color)

    # Set gridlines
    #ax.grid(True, linestyle='--')

    date_form = DateFormatter("%m-%d %H:%M")
    ax.xaxis.set_major_formatter(date_form)


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
        info = json.loads(line)
        times.append(datetime.fromtimestamp(info["time"] / 1000.0))
        co2num = info["co2"]
        if co2num > 10000:
            co2num = 10000
        co2.append(co2num)
        temp.append(info["temp"])
        humidity.append(info["humidity"])

    print("")
    drawGraph(times, co2, "co2", "blue", "co2chart")
    drawGraph(times, temp, "temp", "blue", "tempchart")
    drawGraph(times, humidity, "humidity", "blue", "humiditychart")

except Exception as e:
    print(e)



