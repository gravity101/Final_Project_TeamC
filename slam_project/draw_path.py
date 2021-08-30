import pickle
import numpy as np
import matplotlib.pyplot as plt

if __name__ == '__main__':
    with open("final9.pkl", "rb") as f:
        path = pickle.load(f)

    # draw path
    plt.plot(path['x'], path['y'])
    plt.show()
    # draw heading angle
    plt.plot(path['yaw'], '.')
    plt.show()


"""
import pickle
import numpy as np
import matplotlib.pyplot as plt

if __name__ == '__main__':
    with open("final7.pkl", "rb") as f:
        path = pickle.load(f)

    steps = 50
    last = 4701
    plt.plot(path['x'][:last:steps], path['y'][:last:steps], '.')
    for i in range(0, len(path['x'][:last]), steps):
        plt.text(path['x'][i], path['y'][i], i)
    plt.axis("equal")
    plt.show()
    # print(path['yaw'])
    # plt.plot(path['yaw'], '.-')

    # plt.axis("equal")
    # plt.show()
"""