import matplotlib.pyplot as plt
import numpy as np

# Some example data to display
x = np.linspace(0, 2 * np.pi, 400)
y = np.sin(x ** 2)
x1 = np.linspace(0, -2 * np.pi, 400)
y1 = np.cos(x ** 2)

fig, axs = plt.subplots(2, 2)
axs[0, 0].plot(x, y)
axs[0, 0].set_title("main")
axs[1, 0].plot(x, y**2)
axs[1, 0].set_title("shares x with main")
axs[1, 0].sharex(axs[0, 0])
axs[0, 1].plot(x + 1, y + 1)
axs[0, 1].set_title("unrelated")
axs[0, 1].sharex(axs[0, 0])
axs[1, 1].plot(x + 2, y + 2)
axs[1, 1].set_title("also unrelated")
axs[1, 1].sharex(axs[0, 0])
fig.tight_layout()
plt.show()
