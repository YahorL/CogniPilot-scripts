# Data Processing and Plotting Script

This script processes protobuf data files and generates comprehensive plots of flight data including yaw, pitch, roll, and positions. It also exports data to CSV files for further analysis.

## Features

- **Euler Angles**: Plots yaw, pitch, and roll angles calculated from quaternions
- **Position Data**: Plots X, Y, Z position over time
- **IMU Data**: Plots angular velocity and linear acceleration
- **Magnetic Field**: Plots magnetic field measurements
- **Overview**: Combined plots showing key flight parameters and XY trajectory
- **CSV Export**: Exports IMU, magnetometer, and odometry data to CSV files for external analysis

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
- `--output, -o`: Output directory for plots and CSV files (default: auto-generated from input filename)
- `--csv-only`: Only save CSV files, skip plotting

### Examples

```bash
# Process default data file
python3 process_and_plot_data.py

# Process specific file with custom output directory
python3 process_and_plot_data.py --input my_flight_data.pb --output my_plots

# Process file in current directory
python3 process_and_plot_data.py --input ./data_flight.pb

# Save only CSV files without generating plots
python3 process_and_plot_data.py --input data_flight.pb --csv-only

# Process file and save to custom directory
python3 process_and_plot_data.py --input data_flight.pb --output flight_analysis
```

## Output

The script generates the following files in the specified output directory:

### Plots (PNG files)
1. **euler_angles.png** - Yaw, pitch, and roll angles over time
2. **position.png** - X, Y, Z position over time
3. **angular_velocity.png** - IMU angular velocity measurements
4. **linear_acceleration.png** - IMU linear acceleration measurements
5. **magnetic_field.png** - Magnetic field measurements
6. **pwm_channels.png** - PWM channel outputs
7. **overview.png** - Combined overview with key parameters and XY trajectory

### CSV Files
1. **imu_data.csv** - IMU data with columns: `timestamp,gyro_x,gyro_y,gyro_z,accel_x,accel_y,accel_z`
2. **magnetometer_data.csv** - Magnetometer data with columns: `timestamp,mag_x,mag_y,mag_z`
3. **imu_q31_data.csv** - High-resolution IMU data with temperature: `timestamp,gyro_x,gyro_y,gyro_z,accel_x,accel_y,accel_z,temperature`
4. **odometry_data.csv** - Position and orientation data: `timestamp,pos_x,pos_y,pos_z,quat_x,quat_y,quat_z,quat_w`
5. **pwm_data.csv** - PWM channel data: `timestamp,channel_0,channel_1,channel_2,channel_3`

### CSV File Format
All CSV files include:
- **timestamp**: Time in seconds (floating point)
- **Units**: 
  - Gyroscope: rad/s
  - Accelerometer: m/s²
  - Magnetometer: μT
  - Position: meters
  - Temperature: °C (for IMU Q31 data)

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