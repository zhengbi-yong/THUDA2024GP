import argparse
import os

import numpy as np


def main():
    parser = argparse.ArgumentParser(description="Load and display a .npz file.")
    parser.add_argument(
        "--path",
        type=str,
        required=True,
        help="The directory path where the .npz file is located.",
    )
    parser.add_argument(
        "--filename", type=str, required=True, help="The filename of the .npz file."
    )

    args = parser.parse_args()

    full_path = os.path.join(args.path, args.filename)

    # 修改这里，添加 allow_pickle=True
    with np.load(full_path, allow_pickle=True) as data:
        print("Arrays in the .npz file:", data.files)
        for file in data.files:
            print(f"Array '{file}':")
            print(data[file])


if __name__ == "__main__":
    main()
