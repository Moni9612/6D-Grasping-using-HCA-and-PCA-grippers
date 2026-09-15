import os
import argparse
import torch
import numpy as np
np.float = float
import open3d as o3d
from PIL import Image
import cv2
import csv
from gsnet import AnyGrasp
from graspnetAPI import GraspGroup
from scipy.spatial.transform import Rotation as R

import pandas as pd
from datetime import datetime

parser = argparse.ArgumentParser()
parser.add_argument('--checkpoint_path', required=True, help='Model checkpoint path')
parser.add_argument('--max_gripper_width', type=float, default=0.1, help='Maximum gripper width (<=0.1m)')
parser.add_argument('--gripper_height', type=float, default=0.03, help='Gripper height')
parser.add_argument('--top_down_grasp', action='store_true', help='Output top-down grasps.')
parser.add_argument('--debug', action='store_true', help='Enable debug mode')
cfgs = parser.parse_args()
cfgs.max_gripper_width = max(0, min(0.1, cfgs.max_gripper_width))


def demo(data_dir):
    anygrasp = AnyGrasp(cfgs)
    anygrasp.load_net()

    # Load data
    colors = np.array(Image.open(os.path.join(data_dir, 'color.png')), dtype=np.float32) / 255.0
    depths = np.array(Image.open(os.path.join(data_dir, 'depth.png')))

    # Camera intrinsics
    fx, fy = 923.78, 922.75
    cx, cy = 624.57, 364.80
    scale = 1000.0

    # Workspace limits
    xmin, xmax = -0.19, 0.17       #0.19   0.12   horizontal  -0.19, 0.17
    ymin, ymax = -0.19, 0.19        #0.02   0.15  vertical    
    zmin, zmax = 0.0, 1.0
    lims = [xmin, xmax, ymin, ymax, zmin, zmax]

    # Generate point cloud
    xmap, ymap = np.arange(depths.shape[1]), np.arange(depths.shape[0])
    xmap, ymap = np.meshgrid(xmap, ymap)
    points_z = depths / scale
    points_x = (xmap - cx) / fx * points_z
    points_y = (ymap - cy) / fy * points_z

    mask = (points_z > 0) & (points_z < 1)
    points = np.stack([points_x, points_y, points_z], axis=-1)
    points = points[mask].astype(np.float32)
    colors = colors[mask].astype(np.float32)

    print(points.min(axis=0), points.max(axis=0))

    gg, cloud = anygrasp.get_grasp(
        points, colors,
        lims=lims,
        apply_object_mask=True,
        dense_grasp=False,
        collision_detection=True
    )

    if len(gg) == 0:
        print('No Grasp detected after collision detection!')
        return

    gg = gg.nms().sort_by_score()
    gg_pick = gg[0:20]
    print(gg_pick.scores)
    print('grasp score:', gg_pick[0].score)

    # 2D visualization on color image
    color_img = cv2.imread(os.path.join(data_dir, 'color.png'))

    # Intrinsic matrix
    K = np.array([[fx, 0, cx],
                  [0, fy, cy],
                  [0,  0,  1]])

    for grasp in gg_pick:
        center = grasp.translation
        rotation = grasp.rotation_matrix  # Fixed line
        axis_x = rotation[:, 0]           # Jaw direction

        p1 = center - axis_x * 0.02
        p2 = center + axis_x * 0.02

        def project(pt3d):
            pt = K @ pt3d
            return int(pt[0] / pt[2]), int(pt[1] / pt[2])

        try:
            x1, y1 = project(p1)
            x2, y2 = project(p2)
            cv2.line(color_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
        except:
            continue

    output_file = '/app/anygrasp_sdk/grasp_detection/example_data/top_20_grasp_poses.csv'
    with open(output_file, mode='w', newline='') as file:
        writer = csv.writer(file)
        print(f"Saving top 50 grasp poses to: {output_file}")
        writer.writerow(['Grasp ID', 'Translation', 'Quaternion', 'Score'])
        for i, grasp in enumerate(gg):
            translation = grasp.translation
            rotation_matrix = grasp.rotation_matrix
                
            # Rotation transformations
            B = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])
            C = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]])
            rotation_matrix1 = np.dot(rotation_matrix, B)
            rotation_matrix2 = np.dot(rotation_matrix1, C)
            rotation = R.from_matrix(rotation_matrix2).as_quat()

            score = grasp.score  # ✅ This is the grasp’s confidence score
                
            writer.writerow([i + 1, ','.join(map(str, translation)), ','.join(map(str, rotation)), str(score)])

    # Save result
    save_path = os.path.join(data_dir, 'color_with_grasps.png')
    cv2.imwrite(save_path, color_img)
    print(f"Grasp overlay saved to {save_path}")

    # additonal save path for grasp overlay with timestamp
    new_dir = '/app/anygrasp_sdk/grasp_detection/data'
    os.makedirs(new_dir, exist_ok=True)

    ts_img = datetime.now().strftime("%Y%m%d_%H%M%S")
    new_path = os.path.join(new_dir, f'color_with_grasps_{ts_img}.png')

    cv2.imwrite(new_path, color_img)
    print(f"Grasp overlay saved to {new_path}")

    # ----------------  ( 3 )  OCCUPANCY-MEASUREMENT --------------- #
    # ** inserted block starts here **
    color1 = cv2.imread('/app/anygrasp_sdk/grasp_detection/example_data/color.png')
    depth1 = cv2.imread('/app/anygrasp_sdk/grasp_detection/example_data/depth.png', -1)
    mask = cv2.imread('/app/anygrasp_sdk/grasp_detection/example_data/workspace_mask.png', 0)

    _, thresh = cv2.threshold(mask, 1, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        largest = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest)

        # Horizontal and vertical trims
        cut_x = int(w * 0.053)        #0.01
        cut_y = int(h * 0.025)        #0.04

        # Adjusted ROI
        x_new, w_new = x + cut_x, w - 2 * cut_x
        y_new, h_new = y + cut_y, h - 2 * cut_y

        # Create refined mask and apply both horizontal and vertical trimming
        refined = np.zeros_like(mask)
        cv2.drawContours(refined, [largest], -1, 255, thickness=-1)
        refined[:, :x_new] = 0
        refined[:, x_new + w_new:] = 0
        refined[:y_new, :] = 0
        refined[y_new + h_new:, :] = 0

        depth_mask = (depth1 > 250) & (depth1 < 525) # 380, 479 ,489 latest
        final_mask = depth_mask & (refined > 0)

        area_px = np.count_nonzero(refined)
        obj_px = np.count_nonzero(final_mask)
        occ = (obj_px / area_px) * 100 if area_px else 0

        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        out_dir = '/app/anygrasp_sdk/grasp_detection/occupancy'
        os.makedirs(out_dir, exist_ok=True)

        mask_img = (final_mask * 255).astype(np.uint8)
        mask_path = os.path.join(out_dir, f"object_mask_{ts}.png")
        cv2.imwrite(mask_path, mask_img)

        # Save to Excel
        excel_path = os.path.join(out_dir, "occupancy_log_1.xlsx")
        new_data = pd.DataFrame([{
            "Timestamp": ts,
            "Object Pixels (in refined workspace)": obj_px,
            "Workspace Pixels (total considered area)": area_px,
            "Occupancy": round(occ, 2)
        }])

        if os.path.exists(excel_path):
            old_data = pd.read_excel(excel_path)
            updated_data = pd.concat([old_data, new_data], ignore_index=True)
        else:
            updated_data = new_data

        updated_data.to_excel(excel_path, index=False, engine='openpyxl')


        print(f"Saved occupancy mask   : {mask_path}")
        print(f"Appended occupancy data: {excel_path}")
    else:
        print("No contours found in workspace mask; occupancy not computed.")

    # Delete unnecessary files after saving CSV
    data_dir = '/app/anygrasp_sdk/grasp_detection/example_data'
    files_to_delete = ['color.png', 'depth.png']
    for filename in files_to_delete:
        file_path = os.path.join(data_dir, filename)
        if os.path.exists(file_path):
            os.remove(file_path)
            print(f"Deleted file: {file_path}")
        else:
            print(f"File not found, skipping: {file_path}")


if __name__ == '__main__':
    demo('/app/anygrasp_sdk/grasp_detection/example_data')