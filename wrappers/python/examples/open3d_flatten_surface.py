#importing libraries
import open3d as o3d
import numpy as np
from matplotlib import pyplot as plt 

# Load the mesh 
mesh = o3d.io.read_triangle_mesh("/home/tron/librealsense/1.stl")
o3d.visualization.draw_geometries([mesh]) 
# Compute the vertex colors based on the y-coordinate
y_coords = np.asarray(mesh.vertices)[:, 1]
normalized_y_coords = (y_coords - np.min(y_coords)) / (np.max(y_coords) - np.min(y_coords))

# Define color ranges from top to bottom
top_color_range = [0, 0.10] 
middle_color_range = [0.10, 0.70] 
bottom_color_range = [0.70, 1.0]

# Split the mesh based on color ranges
indices_top = np.where(np.logical_and(normalized_y_coords >= top_color_range[0], normalized_y_coords <= top_color_range[1]))[0]
indices_middle = np.where(np.logical_and(normalized_y_coords >= middle_color_range[0], normalized_y_coords <= middle_color_range[1]))[0]
indices_bottom = np.where(np.logical_and(normalized_y_coords >= bottom_color_range[0], normalized_y_coords <= bottom_color_range[1]))[0]

# Extract vertices and triangles for each region
mesh_top = mesh.select_by_index(indices_top)
mesh_middle = mesh.select_by_index(indices_middle)
mesh_bottom = mesh.select_by_index(indices_bottom)

# Visualize the three mesh parts
o3d.visualization.draw_geometries([mesh_top, mesh_middle, mesh_bottom])
o3d.visualization.draw_geometries([mesh_middle])

# Get the vertices and triangles of the mesh
vertices = np.asarray(mesh_middle.vertices)
triangles = np.asarray(mesh_middle.triangles)

# Choose the axis along which you want to flatten the mesh (e.g., the z-axis)
axis_to_flatten = 2  # 0 for x-axis, 1 for y-axis, 2 for z-axis

# Create a KDTree for spatial search
kdtree = o3d.geometry.KDTreeFlann(mesh_middle)

# Calculate the mean displacement for each vertex
mean_displacements = np.zeros(len(vertices))

for i in range(len(vertices)):
    # Find the neighbors of the current vertex within a small radius
    [k, neighbors_idx, _] = kdtree.search_knn_vector_3d(vertices[i], 10)
    neighbors = vertices[neighbors_idx]

    # Calculate the mean displacement along the chosen axis
    mean_displacement = np.mean(neighbors[:, axis_to_flatten])

    # Store the mean displacement for the current vertex
    mean_displacements[i] = mean_displacement

# Create a new set of vertices with the mean displacement subtracted along the chosen axis
vertices_flattened = vertices.copy()
vertices_flattened[:, axis_to_flatten] -= mean_displacements

# Create a new mesh with the flattened vertices
flattened_mesh = o3d.geometry.TriangleMesh()
flattened_mesh.vertices = o3d.utility.Vector3dVector(vertices_flattened)
flattened_mesh.triangles = o3d.utility.Vector3iVector(triangles)

# Visualize the original and flattened meshes
o3d.visualization.draw_geometries([mesh_middle,flattened_mesh])
o3d.visualization.draw_geometries([flattened_mesh])

# Define the cropping margins (in percentage of the mesh size)
top_margin = 0.1  # 10% from the top
bottom_margin = 0.1  # 10% from the bottom
left_margin = 0.1  # 10% from the left
right_margin = 0.1  # 10% from the right

# Calculate the bounding box coordinates based on cropping margins
min_bound = np.min(vertices_flattened, axis=0) + np.array([left_margin, bottom_margin, 0])
max_bound = np.max(vertices_flattened, axis=0) - np.array([right_margin, top_margin, 0])

# Create a bounding box using Open3D
crop_box = o3d.geometry.AxisAlignedBoundingBox(min_bound, max_bound)

# Crop the flattened mesh using the bounding box
cropped_mesh = flattened_mesh.crop(crop_box)

# Visualize the original, flattened, and cropped meshes
o3d.visualization.draw_geometries([ cropped_mesh])