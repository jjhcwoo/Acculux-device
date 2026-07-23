import matplotlib.pyplot as plt
import numpy as np

from test import run_simulation


trajectory = np.array(
    run_simulation()
)


fig = plt.figure()

ax = fig.add_subplot(
    111,
    projection="3d"
)

ax.plot(
    trajectory[:,0],
    trajectory[:,1],
    trajectory[:,2]
)

plt.show()