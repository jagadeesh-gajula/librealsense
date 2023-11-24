import open3d as o3d

# Read the point cloud file
pcd = o3d.io.read_point_cloud("1.ply")

# Estimate the normals for the point cloud
pcd.estimate_normals()

radius = 0.2

print("normals estimated")

# Create a triangle mesh from the point cloud
mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
    pcd,
    o3d.utility.DoubleVector([radius, radius * 2]))

# Visualize the triangle mesh
o3d.visualization.draw_geometries([mesh])