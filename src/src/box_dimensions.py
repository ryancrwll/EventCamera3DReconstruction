import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

# ==========================================
# CONFIGURATION TOGGLES
# ==========================================
STRUCTURAL_THICKNESS = 0.002 # Strict 2mm extraction for Walls and Floor
SLICE_BUFFER = 0.005         # 5mm safety buffer for the bounding volume cuts
MIN_Z_DEPTH = 0.550          # Kills points closer than 55cm to the camera lens
# ==========================================

def get_acute_angle(model1, model2):
    n1 = np.array(model1[:3])
    n2 = np.array(model2[:3])
    n1 = n1 / np.linalg.norm(n1)
    n2 = n2 / np.linalg.norm(n2)
    dot = np.clip(np.abs(np.dot(n1, n2)), 0.0, 1.0)
    return np.degrees(np.arccos(dot))

def get_distance_to_plane(points, plane_model):
    a, b, c, d = plane_model
    pts = np.asarray(points)
    norm = np.sqrt(a**2 + b**2 + c**2)
    return np.abs((pts[:, 0]*a + pts[:, 1]*b + pts[:, 2]*c + d) / norm)

def orient_plane_to_point(plane_model, target_point):
    """ Flips the plane equation so the normal vector always points TOWARDS the target point (Centroid) """
    a, b, c, d = plane_model
    evaluation = a*target_point[0] + b*target_point[1] + c*target_point[2] + d
    if evaluation < 0:
        return [-a, -b, -c, -d]
    return [a, b, c, d]

def slice_above_floor(pcd, plane_model, buffer=0.0):
    a, b, c, d = plane_model
    if b < 0: a, b, c, d = -a, -b, -c, -d
    pts = np.asarray(pcd.points)
    evals = pts[:, 0]*a + pts[:, 1]*b + pts[:, 2]*c + d
    mask = evals >= -buffer
    return pcd.select_by_index(np.where(mask)[0])

def slice_camera_side(pcd, plane_model, buffer=0.0):
    a, b, c, d = plane_model
    pts = np.asarray(pcd.points)
    evals = pts[:, 0]*a + pts[:, 1]*b + pts[:, 2]*c + d
    mask = evals >= -buffer if d > 0 else evals <= buffer
    return pcd.select_by_index(np.where(mask)[0])

def analyze_box_metrology(ply_path):
    print(f"-> Loading Point Cloud: {ply_path}")
    pcd = o3d.io.read_point_cloud(ply_path)

    if pcd.is_empty():
        return print("Error: Point cloud is empty.")

    print("-> Cleaning Point Cloud...")
    pcd = pcd.voxel_down_sample(voxel_size=0.002)
    pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    pcd.transform(np.array([[-1,0,0,0], [0,-1,0,0], [0,0,1,0], [0,0,0,1]]))

    # ==========================================
    # 0. NEAR-FIELD Z-CROP
    # ==========================================
    pts = np.asarray(pcd.points)
    pcd = pcd.select_by_index(np.where(pts[:, 2] > MIN_Z_DEPTH)[0])

    # ==========================================
    # 1. FIND STRUCTURAL PLANES (2mm Strict)
    # ==========================================
    print("-> Extracting Floor and Walls...")
    working_cloud = pcd
    candidates = []

    for _ in range(15):
        if len(working_cloud.points) < 500: break
        model, inliers = working_cloud.segment_plane(distance_threshold=STRUCTURAL_THICKNESS, ransac_n=3, num_iterations=10000)
        plane_cloud = working_cloud.select_by_index(inliers)

        a, b, c, d = model
        y_comp = abs(b) / np.sqrt(a**2 + b**2 + c**2)
        candidates.append({'model': model, 'cloud': plane_cloud, 'y_comp': y_comp, 'name': 'Unknown'})
        working_cloud = working_cloud.select_by_index(inliers, invert=True)

    floor = None
    walls = []

    for cand in candidates:
        if cand['y_comp'] > 0.70:
            floor = cand; floor['name'] = "Floor (Red)"; break

    if not floor: return print("Error: Could not find floor!")

    for cand in candidates:
        if cand is floor: continue
        angle_to_floor = get_acute_angle(floor['model'], cand['model'])
        if 70.0 <= angle_to_floor <= 110.0:
            if len(walls) == 0:
                cand['name'] = "Wall 1 (Dark Gray)"; walls.append(cand)
            elif len(walls) == 1:
                angle_to_w1 = get_acute_angle(walls[0]['model'], cand['model'])
                if 60.0 <= angle_to_w1 <= 120.0:
                    cand['name'] = "Wall 2 (Light Gray)"; walls.append(cand); break

    floor['cloud'].paint_uniform_color([0.8, 0.1, 0.1])
    if len(walls) > 0: walls[0]['cloud'].paint_uniform_color([0.2, 0.2, 0.2])
    if len(walls) > 1: walls[1]['cloud'].paint_uniform_color([0.4, 0.4, 0.4])

    for cand in candidates:
        if cand is not floor and all(cand is not w for w in walls):
            working_cloud += cand['cloud']

    # ==========================================
    # 2. SLICE OPPOSITE SIDE (Background Delete)
    # ==========================================
    print("-> Removing points behind walls and under floor...")
    working_cloud = slice_above_floor(working_cloud, floor['model'], buffer=0.0)
    for w in walls:
        working_cloud = slice_camera_side(working_cloud, w['model'], buffer=0.0)

    # ==========================================
    # 3. ISOLATE THE BOX
    # ==========================================
    print("-> Isolating the physical Box...")
    labels = np.array(working_cloud.cluster_dbscan(eps=0.02, min_points=50, print_progress=False))
    valid_labels = labels[labels >= 0]
    if len(valid_labels) == 0: return print("Error: No box found.")

    largest_cluster_idx = np.argmax(np.bincount(valid_labels))
    box_cloud = working_cloud.select_by_index(np.where(labels == largest_cluster_idx)[0])

    # ==========================================
    # 4. EXTRACT BOX FACES (With Anti-Crust Filter)
    # ==========================================
    print("-> Extracting metrology faces...")
    box_candidates = []
    temp_box = box_cloud

    for _ in range(12):
        if len(temp_box.points) < 30: break
        model, inliers = temp_box.segment_plane(distance_threshold=0.003, ransac_n=3, num_iterations=10000)
        box_candidates.append({'model': model, 'cloud': temp_box.select_by_index(inliers)})
        temp_box = temp_box.select_by_index(inliers, invert=True)

    valid_faces = []
    top_face = None
    side_faces = []

    for cand in box_candidates:
        # THE FIX: Anti-Crust Filter
        # If this candidate is closer than 2.5cm to any background wall, it is crust. Reject it!
        pts = np.asarray(cand['cloud'].points)
        is_crust = False
        for w in walls:
            if np.mean(get_distance_to_plane(pts, w['model'])) < 0.025:
                is_crust = True; break
        if is_crust: continue

        angle_to_floor = get_acute_angle(floor['model'], cand['model'])

        if angle_to_floor < 25.0 and top_face is None:
            cand['name'] = "Top Face (Green)"; cand['color'] = [0.1, 0.8, 0.1]
            top_face = cand; valid_faces.append(cand)
        elif 65.0 < angle_to_floor < 115.0:
            if len(side_faces) == 0:
                cand['name'] = "Side Face 1 (Blue)"; cand['color'] = [0.1, 0.1, 0.8]
                side_faces.append(cand); valid_faces.append(cand)
            elif len(side_faces) == 1:
                angle_to_s1 = get_acute_angle(side_faces[0]['model'], cand['model'])
                if 70.0 < angle_to_s1 < 110.0:
                    cand['name'] = "Side Face 2 (Yellow)"; cand['color'] = [0.8, 0.8, 0.1]
                    side_faces.append(cand); valid_faces.append(cand); break

    # ==========================================
    # 5. HALF-SPACE INTERSECTION (The Bounding Volume)
    # ==========================================
    print("-> Applying 6-Plane Convex Intersection to chop bounds...")
    all_6_planes = [floor['model']] + [w['model'] for w in walls] + [f['model'] for f in valid_faces]

    # THE FIX: True Centroid Anchor
    # Drop a plumb line from the top face to the floor to find the dead-center of the physical box
    if top_face and floor:
        top_centroid = np.mean(np.asarray(top_face['cloud'].points), axis=0)
        fa, fb, fc, fd = floor['model']
        fn = np.array([fa, fb, fc])
        fn = fn / np.linalg.norm(fn)
        if fn[1] < 0: fn = -fn # Ensure normal points UP
        dist_to_floor = get_distance_to_plane([top_centroid], floor['model'])[0]
        true_centroid = top_centroid - fn * (dist_to_floor * 0.5)
    else:
        true_centroid = np.mean(np.asarray(box_cloud.points), axis=0)

    # Orient all planes to point INWARD toward the true center
    oriented_planes = [orient_plane_to_point(p, true_centroid) for p in all_6_planes]

    box_pts = np.asarray(box_cloud.points)
    is_inside_mask = np.ones(len(box_pts), dtype=bool)

    # A point only survives if it sits on the "inside" of ALL 6 planes
    for a, b, c, d in oriented_planes:
        evals = box_pts[:, 0]*a + box_pts[:, 1]*b + box_pts[:, 2]*c + d
        is_inside_mask &= (evals >= -SLICE_BUFFER)

    # Isolate the completely clean, razor-sharp box
    clean_box_cloud = box_cloud.select_by_index(np.where(is_inside_mask)[0])

    # Paint the discarded crust gray
    leftover_cloud = box_cloud.select_by_index(np.where(~is_inside_mask)[0])
    leftover_cloud.paint_uniform_color([0.5, 0.5, 0.5])

    # Color the clean box sides
    if len(valid_faces) > 0:
        clean_pts = np.asarray(clean_box_cloud.points)
        dist_matrix = np.zeros((len(clean_pts), len(valid_faces)))

        for idx, face in enumerate(valid_faces):
            dist_matrix[:, idx] = get_distance_to_plane(clean_pts, face['model'])

        closest_face_indices = np.argmin(dist_matrix, axis=1)

        for idx, face in enumerate(valid_faces):
            face_pts_idx = np.where(closest_face_indices == idx)[0]
            face_cloud = clean_box_cloud.select_by_index(face_pts_idx)
            face_cloud.paint_uniform_color(face['color'])
            face['cloud'] = face_cloud

    # ==========================================
    # 6. FULL 3D DIMENSION REPORT
    # ==========================================
    print("\n============================================")
    print(" FULL BOX DIMENSIONS (L x W x H)")
    print("============================================")

    if top_face and floor and 'cloud' in top_face and len(top_face['cloud'].points) > 0:
        h = np.mean(get_distance_to_plane(top_face['cloud'].points, floor['model']))
        print(f" Height  (Floor <-> {top_face['name']:<20}) : {h*100:6.2f} cm")

    for face in side_faces:
        if 'cloud' not in face or len(face['cloud'].points) == 0: continue

        parallel_wall = None
        min_angle = 90.0

        for w in walls:
            ang = get_acute_angle(face['model'], w['model'])
            if ang < min_angle:
                min_angle = ang
                if ang < 25.0:  # Match to parallel wall
                    parallel_wall = w

        if parallel_wall:
            dist = np.mean(get_distance_to_plane(face['cloud'].points, parallel_wall['model']))
            print(f" Measure ({parallel_wall['name']:<19} <-> {face['name']:<20}) : {dist*100:6.2f} cm")
        else:
            print(f" Distance for {face['name']} : [No parallel wall found]")

    print("============================================\n")

    # ==========================================
    # 7. MATPLOTLIB BOX-AND-WHISKER PLOT
    # ==========================================
    plot_data, plot_labels, plot_colors = [], [], []

    if floor:
        plot_data.append(get_distance_to_plane(floor['cloud'].points, floor['model']) * 1000)
        plot_labels.append(floor['name']); plot_colors.append('#CC1111')

    for w in walls:
        plot_data.append(get_distance_to_plane(w['cloud'].points, w['model']) * 1000)
        plot_labels.append(w['name']); plot_colors.append('#666666' if "1" in w['name'] else '#999999')

    for face in valid_faces:
        if 'cloud' not in face or len(face['cloud'].points) == 0: continue
        plot_data.append(get_distance_to_plane(face['cloud'].points, face['model']) * 1000)
        plot_labels.append(face['name'])
        hex_color = "#{:02x}{:02x}{:02x}".format(int(face['color'][0]*255), int(face['color'][1]*255), int(face['color'][2]*255))
        plot_colors.append(hex_color)

    if plot_data:
        plt.figure(figsize=(10, 5))
        bplot = plt.boxplot(plot_data, tick_labels=plot_labels, vert=False, patch_artist=True,
                            flierprops=dict(marker='.', markerfacecolor='black', markersize=3, alpha=0.3))

        for patch, color in zip(bplot['boxes'], plot_colors):
            patch.set_facecolor(color); patch.set_alpha(0.8)

        plt.title('Point Cloud Metrology: Absolute Distance to Assigned Mathematical Plane')
        plt.xlabel('Distance from Perfect Mathematical Plane (mm)')
        plt.grid(axis='x', alpha=0.5)
        plt.tight_layout()
        plt.show()

    # Visualization
    geometries = [floor['cloud']] + [w['cloud'] for w in walls] + [f['cloud'] for f in valid_faces if 'cloud' in f] + [leftover_cloud]
    o3d.visualization.draw_geometries(geometries, window_name="Bounding Volume Metrology", width=1280, height=720)

if __name__ == "__main__":
    analyze_box_metrology("datasets/laser_scan.ply")