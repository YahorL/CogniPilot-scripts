import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize

def load_mag_data_csv(filename):
    return np.genfromtxt(filename, delimiter=',', skip_header=1)

def residual_nm(params, data):
    # Unpack parameters
    R_upper = np.array([
        [params[0], params[1], params[2]],
        [0,         params[3], params[4]],
        [0,         0,         params[5]]
    ])
    h = np.array(params[6:9])
    
    residuals = []
    for y in data:
        u = y - h
        val = 1.0 - np.linalg.norm(R_upper @ u)**2
        residuals.append(val**2)
    return np.sum(residuals)

def get_R_and_h(params):
    R = np.array([
        [params[0], params[1], params[2]],
        [0,         params[3], params[4]],
        [0,         0,         params[5]]
    ])
    h = np.array(params[6:9])
    return R, h

def calibrate_nm(data):
    # Initial guess
    R_init = np.eye(3)[np.triu_indices(3)]
    h_init = np.mean(data, axis=0)
    x0 = np.concatenate([R_init, h_init])

    # Optimization
    result = minimize(residual_nm, x0, args=(data,), method='BFGS')
    R, h = get_R_and_h(result.x)
    return R, h

def main(filename):
    data = load_mag_data_csv(filename)
    R, h = calibrate_nm(data)

    # Apply calibration
    calibrated = np.dot((data - h), R.T)

    # Plotting
    fig = plt.figure(figsize=(12, 6))
    ax1 = fig.add_subplot(121, projection='3d')
    ax1.scatter(*data.T, s=5, alpha=0.5, color='blue')
    ax1.set_title('Raw Magnetometer Data')
    ax1.set_xlabel('X'); ax1.set_ylabel('Y'); ax1.set_zlabel('Z')

    ax2 = fig.add_subplot(122, projection='3d')
    ax2.scatter(*calibrated.T, s=5, alpha=0.5, color='green')
    ax2.set_title('Calibrated Magnetometer Data (Suboptimal NM)')
    ax2.set_xlabel('X'); ax2.set_ylabel('Y'); ax2.set_zlabel('Z')
    ax2.set_xlim(-1.5, 1.5); ax2.set_ylim(-1.5, 1.5); ax2.set_zlim(-1.5, 1.5)
    ax2.axis('equal')

    plt.tight_layout()
    plt.show()

    # Print in C-style
    def format_c_array(mat, name):
        if len(mat.shape) == 1:
            arr_str = ', '.join([f'{x:.4f}' for x in mat])
            return f'double {name}[3] = {{{arr_str}}};'
        elif mat.shape == (3, 3):
            arr_str = ',\n    '.join(['{' + ', '.join([f'{x:.4f}' for x in row]) + '}' for row in mat])
            return f'double {name}[3][3] = {{\n    {arr_str}\n}};'

    print(format_c_array(R, 'A'))
    print(format_c_array(h, 'b'))

if __name__ == "__main__":
    main("mag_data/mag_data.csv")
