import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

def get_acute_angle(n1, n2):
    """Calculates the acute angle in degrees between two normal vectors."""
    n1 = n1 / np.linalg.norm(n1)
    n2 = n2 / np.linalg.norm(n2)
    dot = np.clip(np.abs(np.dot(n1, n2)), 0.0, 1.0)
    return np.degrees(np.arccos(dot))

def fit_multiple_planes_robust(ply_path, min_planes=2):
    print(f"-> Loading: {ply_path}")
    pcd = o3d.io.read_point_cloud(ply_path)

    if pcd.is_empty():
        print("Error: Point cloud is empty.")
        return

    # FILTER FRUSTUMS: Remove points that are pure white (the frustums)
    colors = np.asarray(pcd.colors)
    if len(colors) > 0:
        is_frustum = np.all(colors < 0.5, axis=1)
        pcd = pcd.select_by_index(np.where(~is_frustum)[0])

    pcd = pcd.voxel_down_sample(voxel_size=0.002)

    working_cloud = pcd
    planes_found = []

    for i in range(5):
        if len(working_cloud.points) < 200: break

        plane_model, inliers = working_cloud.segment_plane(0.002, 3, 2000)
        inlier_cloud = working_cloud.select_by_index(inliers)

        # ONE UNIFIED BLOCK for processing the plane
        if len(inliers) > 200:
            color = np.random.rand(3)
            inlier_cloud.paint_uniform_color(color)

            # 1. Math Setup
            a, b, c, d = plane_model
            normal = np.array([a, b, c])
            norm = np.linalg.norm(normal)
            normal = normal / norm

            # 2. METROLOGY: Dimension Calculation via Projection
            pts = np.asarray(inlier_cloud.points)

            # Create a basis for the plane
            v1 = np.array([normal[1], -normal[0], 0])
            # Failsafe if normal is exactly along Z-axis
            if np.linalg.norm(v1) < 1e-6:
                v1 = np.array([0, normal[2], -normal[1]])

            v1 = v1 / np.linalg.norm(v1)
            v2 = np.cross(normal, v1)

            # Project points to 2D plane coordinates to get Width/Height
            pts_2d = np.column_stack([np.dot(pts, v1), np.dot(pts, v2)])
            width = np.ptp(pts_2d[:, 0]) * 1000 # mm
            height = np.ptp(pts_2d[:, 1]) * 1000 # mm

            # 3. METROLOGY: Calculate point deviations for the boxplot
            distances = np.abs((pts[:, 0]*a + pts[:, 1]*b + pts[:, 2]*c + d) / norm) * 1000

            planes_found.append({
                'model': plane_model,
                'cloud': inlier_cloud,
                'color': color,
                'label': f"Face {len(planes_found)+1}",
                'dim': (width, height),
                'distances': distances
            })
            print(f"-> Detected {planes_found[-1]['label']}: {width:.1f}x{height:.1f}mm")

        # REMOVE the current plane's inliers from the working cloud ONCE
        working_cloud = working_cloud.select_by_index(inliers, invert=True)

    if len(planes_found) < min_planes:
        print(f"-> Warning: Only found {len(planes_found)} valid planes.")

    # 3. METROLOGY: Print Angles and Distances
    print("\n============================================")
    print(" METROLOGY REPORT")
    print("============================================")
    for i in range(len(planes_found)):
        for j in range(i + 1, len(planes_found)):
            p1, p2 = planes_found[i], planes_found[j]
            angle = get_acute_angle(p1['model'][:3], p2['model'][:3])
            print(f"Angle between {p1['label']} and {p2['label']}: {angle:6.2f} degrees")

            # Parallel distance (if angle < 10 deg)
            if angle < 10.0:
                dist = abs(p1['model'][3] - p2['model'][3]) * 1000
                print(f"  -> Parallel Distance: {dist:6.2f} mm")

    print("============================================\n")

    # Failsafe if no planes were found
    if not planes_found:
        print("No planes found to plot.")
        return

    # 4. Box and Whisker Plot
    plot_data = [f['distances'] for f in planes_found]
    plot_labels = [f['label'] for f in planes_found]
    plot_colors = [f['color'] for f in planes_found]

    plt.figure(figsize=(10, 6))
    bplot = plt.boxplot(plot_data, tick_labels=plot_labels, vert=True, patch_artist=True)

    for patch, color in zip(bplot['boxes'], plot_colors):
        patch.set_facecolor(color)
        patch.set_alpha(0.7)

    plt.title('Metrology: Point Deviation from Fitted Cluster Planes')
    plt.ylabel('Distance to Fitted Plane (mm)')
    plt.grid(axis='y', linestyle='--', alpha=0.7)
    plt.tight_layout()
    plt.show()

    # 5. Visualization
    geometries = [f['cloud'] for f in planes_found]
    o3d.visualization.draw_geometries(geometries, window_name="Multi-Plane Fit")

if __name__ == "__main__":
    fit_multiple_planes_robust("datasets/uw_lego.ply")