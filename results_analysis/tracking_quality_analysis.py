import os
import pandas as pd
import matplotlib.pyplot as plt
from tkinter import Tk, filedialog
from sklearn.metrics import mean_squared_error
from fpdf import FPDF

# Function to select directory using GUI
def select_directory():
    root = Tk()
    root.withdraw()  # Hide the main window
    root.attributes('-topmost', True)  # Bring the dialog to the front
    folder_selected = filedialog.askdirectory()
    return folder_selected

# Load data from CSV files
# Directory selection is already done above
output_directory = select_directory()
thrust_output_path = os.path.join(output_directory, 'NLMPC_thrust_output.csv')
current_setpoint_path = os.path.join(output_directory, 'NLMPC_current_setpoint.csv')
odometry_path = os.path.join(output_directory, 'odometry.csv')

thrust_output = pd.read_csv(thrust_output_path)
current_setpoint = pd.read_csv(current_setpoint_path)
odometry = pd.read_csv(odometry_path)

# Rename columns for easier access
current_setpoint.columns = ['timestamp', 'frame_id', 'pose_heading_position_x', 'pose_heading_position_y', 'pose_heading_position_z', 'pose_heading_velocity_x', 'pose_heading_velocity_y', 'pose_heading_velocity_z', 'pose_heading_heading']
odometry.columns = ['timestamp', 'frame_id', 'odometry_position_x', 'odometry_position_y', 'odometry_position_z', 'odometry_velocity_x', 'odometry_velocity_y', 'odometry_velocity_z', 'odometry_attitude_w', 'odometry_attitude_x', 'odometry_attitude_y', 'odometry_attitude_z', 'odometry_angular_velocity_x', 'odometry_angular_velocity_y', 'odometry_angular_velocity_z']

# Remove duplicate timestamps
current_setpoint = current_setpoint.drop_duplicates(subset=['timestamp'])
odometry = odometry.drop_duplicates(subset=['timestamp'])

# Set timestamps as index
current_setpoint.set_index('timestamp', inplace=True)
odometry.set_index('timestamp', inplace=True)

odometry.index = pd.to_datetime(odometry.index)
current_setpoint.index = pd.to_datetime(current_setpoint.index)

odometry.sort_index(inplace=True)
current_setpoint.sort_index(inplace=True)

# Reindex odometry to match current_setpoint timestamps (interpolation)
odometry_interp = odometry.reindex(current_setpoint.index.union(odometry.index)).interpolate(method='time').reindex(current_setpoint.index)

# Remove NaN values
odometry_interp = odometry_interp.dropna()

# Extract relevant data
pose_x = current_setpoint['pose_heading_position_x']
pose_y = current_setpoint['pose_heading_position_y']
pose_z = current_setpoint['pose_heading_position_z']
odometry_x = odometry_interp['odometry_position_x']
odometry_y = odometry_interp['odometry_position_y']
odometry_z = odometry_interp['odometry_position_z']

# Compute tracking quality metrics (e.g., Mean Squared Error)
mse_x = mean_squared_error(pose_x, odometry_x)
mse_y = mean_squared_error(pose_y, odometry_y)
mse_z = mean_squared_error(pose_z, odometry_z)

# Plot the data
plt.figure(figsize=(12, 8))

# Current setpoint plot
plt.subplot(3, 1, 1)
plt.plot(current_setpoint.index.to_numpy(), pose_x.to_numpy(), label='Setpoint X', color='b')
plt.plot(current_setpoint.index.to_numpy(), odometry_x.to_numpy(), label='Odometry X', color='r', linestyle='--')
plt.xlabel('Time')
plt.ylabel('Position X')
plt.legend()
plt.grid(True)

plt.subplot(3, 1, 2)
plt.plot(current_setpoint.index.to_numpy(), pose_y.to_numpy(), label='Setpoint Y', color='g')
plt.plot(current_setpoint.index.to_numpy(), odometry_y.to_numpy(), label='Odometry Y', color='m', linestyle='--')
plt.xlabel('Time')
plt.ylabel('Position Y')
plt.legend()
plt.grid(True)

plt.subplot(3, 1, 3)
plt.plot(current_setpoint.index.to_numpy(), pose_z.to_numpy(), label='Setpoint Z', color='c')
plt.plot(current_setpoint.index.to_numpy(), odometry_z.to_numpy(), label='Odometry Z', color='k', linestyle='--')
plt.xlabel('Time')
plt.ylabel('Position Z')
plt.legend()
plt.grid(True)

# Save the plots to a PDF file
pdf_path = os.path.join(output_directory, 'tracking_quality_report.pdf')

plt.tight_layout()
plt.savefig(os.path.join(output_directory, 'plots.png'))

# Generate PDF report
pdf = FPDF()
pdf.add_page()
pdf.set_font('Arial', 'B', 16)
pdf.cell(40, 10, 'Tracking Quality Report')

# Add plots to PDF
pdf.image(os.path.join(output_directory, 'plots.png'), x=10, y=20, w=180)

# Add tracking quality metrics
pdf.set_font('Arial', '', 12)
pdf.add_page()
pdf.cell(0, 10, f'Mean Squared Error (X): {mse_x:.4f}')
pdf.ln(10)
pdf.cell(0, 10, f'Mean Squared Error (Y): {mse_y:.4f}')
pdf.ln(10)
pdf.cell(0, 10, f'Mean Squared Error (Z): {mse_z:.4f}')

pdf.output(pdf_path)

print(f'Report generated: {pdf_path}')