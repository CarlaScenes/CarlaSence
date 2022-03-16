import argparse

def align(args):
    # store vloam timestamps to dictionary
    vloam_time = {}
    f = open(args.vloam_poses_timestamps_path, "r")
    lines = f.readlines()
    for l in lines:
        l = l.strip()
        vloam_time[round(float(l), 4)] = 1

    # store ground truth poses
    ground_truth_poses = []
    f = open(args.ground_truth_poses_path, "r")
    lines = f.readlines()
    for l in lines:
        l = l.strip()
        ground_truth_poses.append(l)

    # iterate ground truth poses and keep only those that align with the vloam timestamps
    ground_truth_poses_aligned_poses = []
    f = open(args.ground_truth_timestamps_path, "r")
    lines = f.readlines()
    for c,l in enumerate(lines):
        l = l.strip()
        l = round(float(l), 4)
        if l in vloam_time:
            ground_truth_poses_aligned_poses.append(ground_truth_poses[c])

    # store aligned ground truth poses
    aligned_path = args.ground_truth_poses_path.replace("poses", "aligned_poses")
    textfile = open(aligned_path, "w")
    for p in ground_truth_poses_aligned_poses:
        textfile.write(p + "\n")
    textfile.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description = "Align poses based on timestamp!")
    parser.add_argument("-vlt", "--vloam_poses_timestamps_path", help = "define the path to the vloam timestamps", required=True)
    parser.add_argument("-gnp", "--ground_truth_poses_path", help = "define the path to the ground truth poses", required=True)
    parser.add_argument("-gnt", "--ground_truth_timestamps_path", help = "define the path to the ground truth timestamps", required=True)

    args = parser.parse_args()
    align(args)