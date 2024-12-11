import matplotlib.pyplot as plt
import numpy as np

def plot(x_err):
    x_err = np.array(x_err)
    plt.plot(x_err[:, 0], label=f"w_x")
    plt.plot(x_err[:, 1], label=f"w_y")
    plt.plot(x_err[:, 2], label=f"w_z")
    plt.plot(x_err[:, 3], label=f"v_x")
    plt.plot(x_err[:, 4], label=f"v_y")
    plt.plot(x_err[:, 5], label=f"v_z")

    # Add labels, title, legend, and grid
    plt.xlabel("Time")
    plt.ylabel("Error")
    plt.title("Error over Time")
    plt.legend()
    plt.grid(True)
    plt.savefig(f'results/new_task/new_task_error_over_time.png')
    plt.show()