import json
import traceback
from datetime import datetime
import matplotlib
import numpy as np
from matplotlib import pyplot as plt
from matplotlib.dates import DateFormatter
import matplotlib.ticker as plticker
from tqdm import tqdm
import numpy


def movingAverage(data, window_width):
    cumsum_vec = numpy.cumsum(numpy.insert(data, 0, 0))
    ma_vec = (cumsum_vec[window_width:] - cumsum_vec[:-window_width]) / window_width
    return ma_vec


def drawGraph(times, datalist, title, color, filename=None, base=None):
    fig, ax = plt.subplots(figsize=(26, 16))

    Dates = matplotlib.dates.date2num(times)

    # Set plot title
    ax.set(title=title)

    # Plot graph
    ax.plot(Dates, datalist, linewidth=2, color=color)

    if (base):
        loc = plticker.MultipleLocator(base=base)  # this locator puts ticks at regular intervals
        ax.yaxis.set_major_locator(loc)

    dateLoc = matplotlib.dates.AutoDateLocator()
    ax.xaxis.set_major_locator(dateLoc)

    # Set gridlines
    ax.grid(True, linestyle='--')
    date_form = DateFormatter("%m-%d %H:%M")
    ax.xaxis.set_major_formatter(date_form)

    ##ax.set_xlim([0, 3000])

    if filename is not None:
        fig.savefig(filename + ".png", dpi=fig.dpi, bbox_inches='tight')
    else:
        plt.show()

    print("rendered", title, "graph")


rollingAverage = 20
lineCount = -1

maxTime = input("enter end cut off date in milli epoch time (enter 0 for no cutoff)")
try:
    maxTime = abs(int(maxTime))
except:
    maxTime = 0

try:
    times = []
    co2 = []
    humidity = []
    temp = []
    vpd = []

    file1 = open("environ.csv", 'r')
    Lines = file1.readlines()

    # Strips the newline character
    for line in tqdm(Lines):
        lineCount += 1
        try:
            # print(line)
            info = json.loads(line)
            if (maxTime > 0 and info["time"] > maxTime):
                break

            if len(info) == 5:

                times.append(datetime.fromtimestamp(info["time"] / 1000.0))
                co2num = info["co2"]
                co2.append(min(max(400, co2num), 6000))

                try:
                    v = float(info["vpd"])
                    vpd.append(min(max(0.4, v), 1.5))
                except:
                    vpd.append(-1)

                ##temp
                try:
                    temp.append((min(max(0, float(info["temp"])), 40)))  ###kinda lit, kinda ugly
                except:
                    temp.append(-1)

                ##humidity
                try:
                    h = float(info["humidity"])
                    humidity.append(h)
                except Exception as e:
                    humidity.append(-1)
        except:
            continue
    if not times:
        print("no data to graph")
        exit()

    co2800count = 0
    co2500count = 0
    for i in co2:
        if i >= 800:
            co2800count += 1
        if i >= 500:
            co2500count += 1

    print("co2 > 800ppm ", str(round(co2800count / len(co2) * 100, 3)) + "%", "of the time")
    print("co2 > 500ppm ", str(round(co2500count / len(co2) * 100, 3)) + "%", "of the time")
    print("")

    timeSelection = times[rollingAverage - 1:]  # cut the start off to account for the moving average on values

    drawGraph(timeSelection, movingAverage(co2, rollingAverage), "co2 (ppm)", "blue", "co2chart", 200.0)
    drawGraph(timeSelection, movingAverage(temp, rollingAverage), "Temperature (C)", "blue", "tempchart",
              1.0)
    drawGraph(timeSelection, movingAverage(humidity, rollingAverage), "Humidity (RH %)", "blue",
              "humiditychart", 5.0)

    drawGraph(timeSelection, movingAverage(vpd, rollingAverage), "VPD (Kpa)", "blue", "vpdchart", 0.1)

    if (maxTime > 0):
        print(lineCount)
        with open("environ_truncated.csv", 'w') as truncfile:
            truncfile.writelines(Lines[:lineCount])

    input("\nPress enter to exit")

except Exception as e:
    print(e)
    traceback.print_exc()
