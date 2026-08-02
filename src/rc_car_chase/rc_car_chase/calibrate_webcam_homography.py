"""One-time, ROS-independent calibration utility.

Click ground-plane reference points in the live webcam feed and type in the
matching real-world coordinate (in the robot's odom frame, meters) for each
one. Needs at least 4 point pairs. Get the real-world coordinates either with
a tape measure, or by driving the robot to the marked spot and reading its
current position from a second terminal:

    ros2 topic echo /robot5/odom --field pose.pose.position

Run once whenever the webcam or the robot's power-on position/orientation
changes. This script does not touch ROS at all, so it works regardless of
DDS/network state.

Usage (defaults already point at the right places, --output only needed to override):
    ros2 run rc_car_chase calibrate_webcam_homography --device /dev/video2
"""
import argparse
import datetime
import os

import cv2
import numpy as np
import yaml


class Calibrator:
    def __init__(self, window_name):
        self.window_name = window_name
        self.pixel_points = []
        self.pending_click = None
        self.crosshair = None  # (u, v), set once first frame size is known

    def on_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.pending_click = (x, y)

    def ensure_crosshair(self, frame):
        if self.crosshair is None:
            h, w = frame.shape[:2]
            self.crosshair = [w // 2, h // 2]

    def move_crosshair(self, dx, dy, frame):
        h, w = frame.shape[:2]
        self.crosshair[0] = max(0, min(w - 1, self.crosshair[0] + dx))
        self.crosshair[1] = max(0, min(h - 1, self.crosshair[1] + dy))

    def draw(self, frame):
        vis = frame.copy()
        for i, (u, v) in enumerate(self.pixel_points):
            cv2.circle(vis, (int(u), int(v)), 6, (0, 255, 0), -1)
            cv2.putText(vis, str(i + 1), (int(u) + 8, int(v) - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        if self.crosshair is not None:
            cu, cv_ = self.crosshair
            cv2.drawMarker(vis, (cu, cv_), (0, 0, 255), cv2.MARKER_CROSS, 20, 2)
        cv2.putText(vis, f'points: {len(self.pixel_points)} (need >= 4)  '
                          f'[click OR wasd+space=add  u=undo  c=compute+save  q=quit]',
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
        return vis


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--device', default='/dev/video2')
    parser.add_argument(
        '--output', default='/home/rokey/rokey_ws/src/rc_car_chase/config/webcam_homography.yaml',
    )
    parser.add_argument('--capture-width', type=int, default=1280)
    parser.add_argument('--capture-height', type=int, default=720)
    args = parser.parse_args()

    cap = cv2.VideoCapture(args.device)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.capture_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.capture_height)
    if not cap.isOpened():
        raise RuntimeError(f'Could not open webcam device {args.device}')

    window_name = 'webcam_homography_calibration'
    calib = Calibrator(window_name)
    cv2.namedWindow(window_name)
    cv2.setMouseCallback(window_name, calib.on_mouse)

    world_points = []
    print('Click a ground-plane reference point in the window (or move the red')
    print('crosshair with w/a/s/d, W/A/S/D for bigger steps, and press SPACE to')
    print('add it), then answer the prompt here with its real-world (x, y) in the')
    print('robot odom frame (meters).')
    print("Press 'u' to undo the last point, 'c' to compute+save once you have >= 4,")
    print("'q' to quit without saving.")
    print('NOTE: if you click and nothing seems to happen, check THIS terminal - ')
    print('a text prompt appears here waiting for the real-world coordinate.')

    small_step, big_step = 5, 25
    move_keys = {
        ord('w'): (0, -small_step), ord('s'): (0, small_step),
        ord('a'): (-small_step, 0), ord('d'): (small_step, 0),
        ord('W'): (0, -big_step), ord('S'): (0, big_step),
        ord('A'): (-big_step, 0), ord('D'): (big_step, 0),
    }

    while True:
        ok, frame = cap.read()
        if not ok:
            print('WARNING: frame grab failed')
            continue
        calib.ensure_crosshair(frame)

        if calib.pending_click is not None:
            u, v = calib.pending_click
            # Show a clear on-screen cue BEFORE blocking on terminal input, so the
            # window doesn't just look frozen while input() is waiting elsewhere.
            vis = calib.draw(frame)
            cv2.putText(vis, f'Point at ({u},{v}) - CHECK THE TERMINAL WINDOW for input prompt!',
                        (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            cv2.imshow(window_name, vis)
            cv2.waitKey(1)

            calib.pending_click = None
            try:
                raw = input(f'Point {len(calib.pixel_points) + 1} at pixel ({u},{v}) '
                            f'-> enter real-world "x y" (meters, odom frame): ')
                x_str, y_str = raw.strip().split()
                wx, wy = float(x_str), float(y_str)
            except (ValueError, EOFError):
                print('Invalid input, discarding this point.')
                continue
            calib.pixel_points.append((u, v))
            world_points.append((wx, wy))

        cv2.imshow(window_name, calib.draw(frame))
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            print('Quit without saving.')
            break

        if key == ord('u') and calib.pixel_points:
            calib.pixel_points.pop()
            world_points.pop()
            print('Removed last point.')

        if key == ord(' '):
            calib.pending_click = tuple(calib.crosshair)

        if key in move_keys:
            dx, dy = move_keys[key]
            calib.move_crosshair(dx, dy, frame)

        if key == ord('c'):
            if len(calib.pixel_points) < 4:
                print(f'Need at least 4 points, have {len(calib.pixel_points)}.')
                continue
            pixel_arr = np.array(calib.pixel_points, dtype=np.float32)
            world_arr = np.array(world_points, dtype=np.float32)
            H, _ = cv2.findHomography(pixel_arr, world_arr)
            if H is None:
                print('Homography computation failed - check for collinear points.')
                continue
            out = {
                'homography_matrix': H.flatten().tolist(),
                'frame_id': 'odom',
                'calibrated_at': datetime.datetime.now().isoformat(),
                'num_points': len(calib.pixel_points),
                'pixel_points': [[float(u), float(v)] for u, v in calib.pixel_points],
                'world_points': [[float(x), float(y)] for x, y in world_points],
                'note': ('Homography is only valid for the robot odom origin active '
                         'at calibration time. Recalibrate after any robot reboot / '
                         'odom reset, or if the webcam is moved.'),
            }
            os.makedirs(os.path.dirname(args.output) or '.', exist_ok=True)
            with open(args.output, 'w') as f:
                yaml.safe_dump(out, f, sort_keys=False)
            print(f'Saved homography ({len(calib.pixel_points)} points) to {args.output}')
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
