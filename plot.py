import numpy as np
import matplotlib.pyplot as plt

xyz = np.loadtxt("plotData.csv", delimiter=",")

fig = plt.figure(figsize=(12, 12))
ax = fig.add_subplot(projection='3d')

x = np.linspace(0,100,10)
y = np.linspace(0,100,10)

X,Y = np.meshgrid(x,y)

Z = (X*-19.99518481 + Y*138.4946695 + 204547.2696)/3942.517004

ax.scatter(xyz[:,0], xyz[:,1], xyz[:,2])
surf = ax.plot_surface(X, Y, Z, color='green', alpha=0.5)

plt.show()