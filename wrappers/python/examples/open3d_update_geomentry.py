import numpy as np
import open3d as o3d
import time

# Function to update point cloud data
def update_point_cloud(pcd):
    new_xyz = np.random.rand(100000, 3)
    pcd.points = o3d.utility.Vector3dVector(new_xyz)

# Create a random point cloud
xyz = np.random.rand(100, 3)
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(xyz)

# Set up visualization window
vis = o3d.visualization.Visualizer()
vis.create_window()

# Add point cloud to the window
vis.add_geometry(pcd)

# Get render option for better visualization
render_option = vis.get_render_option()
render_option.point_size = 5.0  # Set the point size

# Run the visualization loop
while True:
    update_point_cloud(pcd)  # Update point cloud data
    vis.update_geometry(pcd)  # Update geometry
    vis.poll_events()
    vis.update_renderer()
    # time.sleep(0.1)  # Wait for 1 second

# Close the visualization window when the loop ends
vis.destroy_window()

