import numpy as np
import open3d as o3d
 
def create_lines(vertices, measurements_per_layer):
    lines = []
    num_vertices = len(vertices)
    layers = num_vertices // measurements_per_layer
    # Connect points within layers
    for layer in range(layers):
        for i in range(measurements_per_layer):
            idx = layer * measurements_per_layer + i
            next_idx = idx + 1 if (i + 1) < measurements_per_layer else layer * measurements_per_layer
            lines.append([idx, next_idx])
    # Connect corresponding points in consecutive layers
    for i in range(measurements_per_layer):
        for layer in range(layers - 1):
            idx = layer * measurements_per_layer + i
            upper_idx = idx + measurements_per_layer
            lines.append([idx, upper_idx])
    return lines
 
if __name__ == "__main__":
    # Read the test data from the file we created
    print("Read in the prism point cloud data (pcd)")
    pcd = o3d.io.read_point_cloud("output.xyz", format="xyz")
    # Visualize the point cloud data
    print("Let's visualize the PCD: (spawns separate interactive window)")
    o3d.visualization.draw_geometries([pcd])
    # Extract vertices
    vertices = np.asarray(pcd.points)
    measurements_per_layer = 32  # Define how many measurements you have per layer
    lines = create_lines(vertices, measurements_per_layer)
    # Create a line set
    line_set = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(vertices),
        lines=o3d.utility.Vector2iVector(lines)
    )
    # Visualize the line set
    print("Visualizing the line set:")
    o3d.visualization.draw_geometries([line_set])
