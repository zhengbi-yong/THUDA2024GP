import argparse
import os

import numpy as np


def main():
    parser = argparse.ArgumentParser(description="Load and display a .npy file.")
    parser.add_argument(
        "--path",
        type=str,
        required=True,
        help="The directory path where the .npy file is located.",
    )
    parser.add_argument(
        "--filename", type=str, required=True, help="The filename of the .npy file."
    )

    args = parser.parse_args()

    full_path = os.path.join(args.path, args.filename)

    # 修改这里，添加 allow_pickle=True
    array = np.load(full_path, allow_pickle=True)
    print(array)


if __name__ == "__main__":
    main()
