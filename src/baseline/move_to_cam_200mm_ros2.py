#!/usr/bin/env python3

from pathlib import Path
import runpy


if __name__ == '__main__':
    runpy.run_path(
        str(Path(__file__).with_name('scripts') / 'move_to_cam_200mm_ros2.py'),
        run_name='__main__',
    )
