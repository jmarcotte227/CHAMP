import time
import pickle
import numpy as np
import cv2
from flir_toolbox import *
import matplotlib.pyplot as plt
from scipy.signal import iirfilter, sosfilt

def flame_detection_aluminum(
    raw_img, threshold=1.0e4, area_threshold=4, percentage_threshold=0.8
):
    """
    flame detection by raw counts thresholding and connected components labeling
    centroids: x,y
    bbox: x,y,w,h
    adaptively increase the threshold to 60% of the maximum pixel value
    """
    threshold = max(threshold, percentage_threshold * np.max(raw_img))
    thresholded_img = (raw_img > threshold).astype(np.uint8)

    _, labels, stats, centroids = cv2.connectedComponentsWithStats(
        thresholded_img, connectivity=4
    )

    valid_indices = np.where(stats[:, cv2.CC_STAT_AREA] > area_threshold)[0][
        1:
    ]  ###threshold connected area
    if len(valid_indices) == 0:
        return None, None

    average_pixel_values = [
        np.mean(raw_img[labels == label]) for label in valid_indices
    ]  ###sorting
    valid_index = valid_indices[
        np.argmax(average_pixel_values)
    ]  ###get the area with largest average brightness value

    # Extract the centroid and bounding box of the largest component
    centroid = centroids[valid_index]
    bbox = stats[valid_index, :-1]

    return centroid, bbox

def line_intersect(p1,v1,p2,v2):
    #calculate the intersection of two lines, on line 1
    #find the closest point on line1 to line2
    w = p1 - p2
    a = np.dot(v1, v1)
    b = np.dot(v1, v2)
    c = np.dot(v2, v2)
    d = np.dot(v1, w)
    e = np.dot(v2, w)

    sc = (b*e - c*d) / (a*c - b*b)
    closest_point = p1 + sc * v1

    return closest_point

def flame_tracking(save_path, robot, robot2, positioner, flir_intrinsic, height_offset=0):
    with open(save_path + "ir_recording.pickle", "rb") as file:
        ir_recording = pickle.load(file)
    ir_ts = np.loadtxt(save_path + "ir_stamps.csv", delimiter=",")
    if ir_ts.shape[0] == 0:
        raise ValueError("No flame detected")
    joint_angle = np.loadtxt(save_path + "weld_js_exe.csv", delimiter=",")
    timeslot = [ir_ts[0] - ir_ts[0], ir_ts[-1] - ir_ts[0]]
    duration = np.mean(np.diff(timeslot))

    flame_3d = []
    job_no = []
    torch_path = []
    for start_time in timeslot[:-1]:
        start_idx = np.argmin(np.abs(ir_ts - ir_ts[0] - start_time))
        end_idx = np.argmin(np.abs(ir_ts - ir_ts[0] - start_time - duration))
    ## including for tracking purposes
    time_diffs = []
    # find all pixel regions to record from flame detection
    for i in range(start_idx, end_idx):
        start_time_tracking = time.perf_counter()
        ir_image = ir_recording[i]
        try:
            centroid, _ = flame_detection_aluminum(ir_image, percentage_threshold=0.8)
        except ValueError:
            centroid = None

        if centroid is not None:
            # find spatial vector ray from camera sensor
            vector = np.array(
                [
                    (centroid[0] - flir_intrinsic["c0"]) / flir_intrinsic["fsx"],
                    (centroid[1] - flir_intrinsic["r0"]) / flir_intrinsic["fsy"],
                    1,
                ]
            )
            vector = vector / np.linalg.norm(vector)
            # find index closest in time of joint_angle
            joint_idx = np.argmin(np.abs(ir_ts[i] - joint_angle[:, 0]))
            robot2_pose_world = robot2.fwd(joint_angle[joint_idx][8:-2], world=True)
            p2 = robot2_pose_world.p
            v2 = robot2_pose_world.R @ vector
            robot1_pose = robot.fwd(joint_angle[joint_idx][2:8])
            p1 = robot1_pose.p
            v1 = robot1_pose.R[:, 2]
            positioner_pose = positioner.fwd(joint_angle[joint_idx][-2:], world=True)

            # find intersection point
            intersection = line_intersect(p1, v1, p2, v2)
            # offset by height_offset
            intersection[2] = intersection[2]+height_offset
            intersection = positioner_pose.R.T @ (intersection - positioner_pose.p)
            torch = positioner_pose.R.T @ (robot1_pose.p - positioner_pose.p)
            time_dif_tracking=time.perf_counter()-start_time_tracking
            time_diffs.append(time_dif_tracking)
            flame_3d.append(intersection)
            torch_path.append(torch)
            job_no.append(int(joint_angle[joint_idx][1]))

    flame_3d = np.array(flame_3d)
    torch_path = np.array(torch_path)
    job_no = np.array(job_no)
    ## processing timing data
    time_diffs=np.array(time_diffs)
    print("Average: ", np.mean(time_diffs))
    print("Worst: ", np.max(time_diffs))
    return flame_3d, torch_path, job_no
