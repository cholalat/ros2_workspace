import numpy as np


def likelihood_fields(z, x, m):
    q = 1
    z_max = 4
    for k in range(len(z)):
        if z[k] < z_max:

            x_ztk = x[0] + z[k] * np.array([np.cos(z[k]), np.sin(z[k])])
            y_ztk = x[1] + z[k] * np.array([np.sin(z[k]), np.cos(z[k])])