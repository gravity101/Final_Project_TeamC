"""
import pickle
import numpy as np
import matplotlib.pyplot as plt

if __name__ == '__main__':
    with open("curvature.pkl", "rb") as f:
        path = pickle.load(f)

    # draw path
    x = max(path['x']) - min(path['x'])
    y = max(path['y']) - min(path['y'])
    print( (x+y)/4 )   # 0.865726392918
    plt.plot(path['x'], path['y'])
    plt.xlim(-1, 2)
    plt.ylim(-2, 1)
    plt.grid(True)
    plt.show()
    # draw heading angle
    plt.plot(path['yaw'], '.')
    plt.show()
"""

"""
import pickle
import numpy as np
import matplotlib.pyplot as plt

if __name__ == '__main__':
    with open("final_0903_half.pkl", "rb") as f:
        path = pickle.load(f)

    steps = 50
    last = 4701
    plt.plot(path['x'][:last:steps], path['y'][:last:steps], '.')
#    for i in range(0, len(path['x'][:last]), steps):
#        plt.text(path['x'][i], path['y'][i], i)
    plt.axis("equal")
    plt.show()
    # print(path['yaw'])
    # plt.plot(path['yaw'], '.-')

    # plt.axis("equal")
    # plt.show()
"""

import pickle
import numpy as np
import matplotlib.pyplot as plt

if __name__ == '__main__':
    with open("final_0916_9.pkl", "rb") as f:
        path = pickle.load(f)

    # draw path
    plt.figure(1)
    plt.plot(path['x'], path['y'])

    # draw heading angle
    plt.figure(2)
    plt.plot(path['yaw'])
    plt.show()
