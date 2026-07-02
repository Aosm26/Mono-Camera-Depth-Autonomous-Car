#!/usr/bin/env python3
import os
import argparse
import numpy as np
import cv2
from pathlib import Path

def parse_args():
    parser = argparse.ArgumentParser(
        description='Selecting video frames for training sc_depth')
    parser.add_argument('--dataset_dir', default='/home/aosm/Mono Camera Depth Autonomuos Car/scdepth_dataset_prep/dataset',
                        help='Path to dataset directory containing training/ folder')
    parser.add_argument('--threshold', type=float, default=0.5,
                        help='Movement ratio threshold (default: 0.5)')
    args = parser.parse_args()
    return args

def compute_movement_ratio(frame1, frame2):
    if frame1 is None or frame2 is None:
        return 0.0
    frame1_gray = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
    frame2_gray = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)
    h, w = frame1_gray.shape
    
    # Cast to int16 to avoid underflow/overflow wrapping in uint8 subtraction
    diff = np.abs(frame1_gray.astype(np.int16) - frame2_gray.astype(np.int16))
    
    # Calculate ratio of pixels that changed by more than 10 intensity levels
    ratio = (diff > 10).sum() / (h * w)
    return ratio

def generate_index(scene_path, threshold):
    # Get all .jpg images sorted numerically
    images = sorted(list(scene_path.glob('*.jpg')))
    if not images:
        print(f"Warning: No .jpg files found in {scene_path}")
        return []

    index = [0]
    for idx in range(1, len(images)):
        frame1 = cv2.imread(str(images[index[-1]]))
        frame2 = cv2.imread(str(images[idx]))

        move_ratio = compute_movement_ratio(frame1, frame2)
        if move_ratio < threshold:
            continue
        index.append(idx)

    print(f"  Total frames: {len(images)} -> Valid frames (above {threshold} movement): {len(index)}")
    return index

def main():
    args = parse_args()
    data_root = Path(args.dataset_dir)
    training_dir = data_root / 'training'

    if not training_dir.exists():
        print(f"Error: Training directory '{training_dir}' does not exist!")
        return

    # Find all scene folders under training
    scenes = sorted([d for d in training_dir.iterdir() if d.is_dir()])
    if not scenes:
        print(f"No scene directories found under {training_dir}")
        return

    for scene in scenes:
        print(f"Processing scene: {scene.name}")
        index = generate_index(scene, args.threshold)
        
        # Save indices to frame_index.txt
        index_file = scene / 'frame_index.txt'
        np.savetxt(index_file, index, fmt='%d', delimiter='\n')
        print(f"  Saved indices to: {index_file}")

if __name__ == '__main__':
    main()
