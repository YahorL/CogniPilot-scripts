# Data Processing and Plotting Script

This script processes protobuf data files and generates comprehensive plots of flight data including yaw, pitch, roll, and positions.

## Features

- **Euler Angles**: Plots yaw, pitch, and roll angles calculated from quaternions
- **Position Data**: Plots X, Y, Z position over time
- **IMU Data**: Plots angular velocity and linear acceleration
- **Magnetic Field**: Plots magnetic field measurements
- **Overview**: Combined plots showing key flight parameters and XY trajectory

## Requirements

The script requires the following Python packages:
- `synapse_pb` - For reading protobuf data
- `delimited_protobuf` - For parsing delimited protobuf messages
- `numpy` - For numerical operations
- `matplotlib` - For plotting
- `scipy` - For quaternion to euler angle conversion

## Installation

Install the required packages:

```bash
pip3 install synapse_pb delimited_protobuf numpy matplotlib scipy --break-system-packages
```

## Usage

### Basic Usage

```bash
python3 process_and_plot_data.py --input log_data/data_flight.pb --output plots
```

### Command Line Options

- `--input, -i`: Input protobuf file path (default: `log_data/data_flight.pb`)
- `--output, -o`: Output directory for plots (default: `plots`)

### Examples

```bash
# Process default data file
python3 process_and_plot_data.py

# Process specific file with custom output directory
python3 process_and_plot_data.py --input my_flight_data.pb --output my_plots

# Process file in current directory
python3 process_and_plot_data.py --input ./data_flight.pb
```

## Output

The script generates the following plots in the specified output directory:

1. **euler_angles.png** - Yaw, pitch, and roll angles over time
2. **position.png** - X, Y, Z position over time
3. **angular_velocity.png** - IMU angular velocity measurements
4. **linear_acceleration.png** - IMU linear acceleration measurements
5. **magnetic_field.png** - Magnetic field measurements
6. **overview.png** - Combined overview with key parameters and XY trajectory

## Data Summary

The script also prints a summary of the processed data including:
- Number of samples for each data type
- Duration of the flight
- Time range of the data

## Data Types Processed

- **IMU**: Angular velocity and linear acceleration
- **Odometry**: Position and orientation (quaternions)
- **Magnetic Field**: Magnetic field measurements
- **PWM**: PWM channel data (if available)
- **Input**: Input channel data (if available)

## Quaternion to Euler Conversion

The script uses the ZYX (yaw-pitch-roll) convention for converting quaternions to Euler angles:
- **Yaw**: Rotation around Z-axis
- **Pitch**: Rotation around Y-axis  
- **Roll**: Rotation around X-axis

## Notes

- The script processes data in chunks and shows progress every 1000 frames
- All plots are saved as high-resolution PNG files (300 DPI)
- The script handles missing data gracefully
- Time is converted from seconds + nanoseconds to floating-point seconds 