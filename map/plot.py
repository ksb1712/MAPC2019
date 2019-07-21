import numpy as np
import matplotlib.pyplot as plt
import glob
from matplotlib import colors


data_list = glob.glob("./*.npy")
# norm = colors.BoundaryNorm(bounds, cmap.N)
l = len(data_list)

fig, axs = plt.subplots(1,l,edgecolor='k')

axs = np.array(axs)
axs = axs.ravel()


for i in range(10000):
    
    for j in range(l):
        # ax = plt.subplot(j+1)
        # plt.subplot(1,l,j+1)
        data = np.load(data_list[j])
        data += 1

        data = data * 10
        axs[j].matshow(data)
        axs[j].set_title(data_list[j][2:-4])
        axs[j].axis('off')
        
        
    
    plt.pause(0.05)

plt.show()