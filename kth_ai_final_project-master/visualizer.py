import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import os, sys

sns.set()

if(len(sys.argv) < 2):
    print("Please specify which map was used as CL argument")
    sys.exit(1)

circuit = str(sys.argv[1])
print(circuit)

files = [circuit, "visionWithConesWaypointsMap.txt", "wallsWithPathMap.txt", ]

prefix = files[0][:-4]

for file in files:
    data = []
    with open(file, 'r') as f:
        for line in f:
            line = list(line)
            data.append(line[:-1])


    data = np.array(data).astype(np.float)

    fig, ax = plt.subplots()
    plot = sns.heatmap(data, cmap = ['white', 'black', 'yellow', 'red', 'red', 'blue', 'orange', 'pink', 'red', 'green'], cbar = False)
    plot.set_title(str(file[:-4]))

    if(not os.path.isdir("viz")):
        os.mkdir("viz")

    if(not os.path.isdir(os.path.join("viz", prefix))):
        os.mkdir(os.path.join("viz", prefix))


    plot.get_figure().savefig(os.path.join("viz", prefix, file[:-3]+"png"))
    plt.show()

