import math
import time
from typing import Sequence

import cv2
import mediapipe as mp


# Runtime and control constants.
CAMERA_INDEX = 0
MAX_NUM_HANDS = 1
MIN_DETECTION_CONFIDENCE = 0.7

KP = 0.5
KI = 0.01
KD = 0.1

DEADZONE_X = 0.05
DEADZONE_Y = 0.05
DEADZONE_A = 0.05

EPSILON = 1e-9


class PID:
    def __init__(self, kp: float, ki: float, kd: float):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.last_error = 0.0
        self.integral = 0.0

    def calculate(self, error: float, dt: float = 0.03) -> float:
        if dt <= 0:
            dt = 1e-3
        self.integral += error * dt
        derivative = (error - self.last_error) / dt
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        self.last_error = error
        return output


def landmark_distance(landmarks: Sequence, idx_a: int, idx_b: int) -> float:
    point_a = landmarks[idx_a]
    point_b = landmarks[idx_b]
    return math.hypot(point_a.x - point_b.x, point_a.y - point_b.y)


def landmark_vector(landmarks: Sequence, idx_a: int, idx_b: int) -> tuple[float, float, float]:
    point_a = landmarks[idx_a]
    point_b = landmarks[idx_b]
    return (
        point_b.x - point_a.x,
        point_b.y - point_a.y,
        point_b.z - point_a.z,
    )


def vec_dot(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def vec_cross(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def vec_norm(v: tuple[float, float, float]) -> float:
    return math.sqrt(vec_dot(v, v))


def vec_normalize(v: tuple[float, float, float]) -> tuple[float, float, float]:
    n = vec_norm(v)
    if n < EPSILON:
        return (0.0, 0.0, 0.0)
    return (v[0] / n, v[1] / n, v[2] / n)


def project_to_plane(
    v: tuple[float, float, float], normal: tuple[float, float, float]
) -> tuple[float, float, float]:
    normal_scale = vec_dot(v, normal)
    return (
        v[0] - normal_scale * normal[0],
        v[1] - normal_scale * normal[1],
        v[2] - normal_scale * normal[2],
    )


def hand_height(landmarks: Sequence) -> float:
    # Approximate distance from camera using hand size in image.
    return landmark_distance(landmarks, 12, 0)


def hand_width(landmarks: Sequence) -> float:
    return landmark_distance(landmarks, 20, 4)


def middle_axis_roll(landmarks: Sequence) -> float:
    """Roll around the middle-finger axis (MCP 9 -> TIP 12)."""
    axis = vec_normalize(landmark_vector(landmarks, 9, 12))
    if vec_norm(axis) < EPSILON:
        return 0.0

    palm_side = project_to_plane(landmark_vector(landmarks, 5, 17), axis)
    palm_side_norm = vec_norm(palm_side)
    if palm_side_norm < EPSILON:
        return 0.0
    palm_side = (
        palm_side[0] / palm_side_norm,
        palm_side[1] / palm_side_norm,
        palm_side[2] / palm_side_norm,
    )

    camera_up = (0.0, -1.0, 0.0)
    reference = project_to_plane(camera_up, axis)
    if vec_norm(reference) < EPSILON:
        camera_right = (1.0, 0.0, 0.0)
        reference = project_to_plane(camera_right, axis)

    reference_norm = vec_norm(reference)
    if reference_norm < EPSILON:
        return 0.0
    reference = (
        reference[0] / reference_norm,
        reference[1] / reference_norm,
        reference[2] / reference_norm,
    )

    sin_term = vec_dot(vec_cross(reference, palm_side), axis)
    cos_term = vec_dot(reference, palm_side)
    return math.atan2(sin_term, cos_term)


def calibrate_hand(
    size_landmarks: Sequence, angle_landmarks: Sequence
) -> tuple[float, float, float]:
    return (
        hand_height(size_landmarks),
        hand_width(size_landmarks),
        middle_axis_roll(angle_landmarks),
    )


def normalize_angle(angle_rad: float) -> float:
    while angle_rad > math.pi:
        angle_rad -= 2.0 * math.pi
    while angle_rad < -math.pi:
        angle_rad += 2.0 * math.pi
    return angle_rad


def compute_errors(
    size_landmarks: Sequence,
    angle_landmarks: Sequence,
    ref_height: float,
    ref_width: float,
    ref_roll: float,
) -> tuple[float, float, float]:
    # Landmark 9 is close to palm center in image space.
    error_x = (0.5 - size_landmarks[9].x) * 2.0

    current_height = hand_height(size_landmarks)
    current_width = hand_width(size_landmarks)
    height_error = (ref_height - current_height) / max(ref_height, 1e-6)
    width_error = (ref_width - current_width) / max(ref_width, 1e-6)
    error_y = 0.5 * (height_error + width_error)

    current_roll = middle_axis_roll(angle_landmarks)
    error_a = normalize_angle(current_roll - ref_roll)
    return error_x, error_y, error_a


def apply_deadzone(value: float, threshold: float) -> float:
    return 0.0 if abs(value) < threshold else value


def draw_middle_axis_overlay(frame, hand_landmarks: Sequence) -> tuple[int, int]:
    frame_height, frame_width, _ = frame.shape
    center_x = int(hand_landmarks[9].x * frame_width)
    center_y = int(hand_landmarks[9].y * frame_height)
    tip_x = int(hand_landmarks[12].x * frame_width)
    tip_y = int(hand_landmarks[12].y * frame_height)

    cv2.line(frame, (center_x, center_y), (tip_x, tip_y), (255, 0, 0), 3)
    cv2.circle(frame, (center_x, center_y), 10, (0, 255, 0), cv2.FILLED)
    cv2.putText(
        frame,
        f"Center: {center_x}, {center_y}",
        (10, 40),
        cv2.FONT_HERSHEY_PLAIN,
        2,
        (0, 255, 0),
        2,
    )
    return center_x, center_y


def main() -> None:
    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        raise RuntimeError("Cannot open camera. Check if it is connected or already in use.")

    print("Camera initialized. Starting hand tracking...")

    mp_hands = mp.solutions.hands
    mp_draw = mp.solutions.drawing_utils

    pid_x = PID(kp=KP, ki=KI, kd=KD)
    pid_y = PID(kp=KP, ki=KI, kd=KD)
    pid_a = PID(kp=KP, ki=KI, kd=KD)

    ref_height = None
    ref_width = None
    ref_roll = None

    last_time = time.time()

    print("Loading MediaPipe Hands model...")
    with mp_hands.Hands(
        static_image_mode=False,
        max_num_hands=MAX_NUM_HANDS,
        min_detection_confidence=MIN_DETECTION_CONFIDENCE,
    ) as hands:
        print("MediaPipe Hands model loaded. Processing video feed...")
        print("Press 's' to calibrate, 'q' to quit.")

        while cap.isOpened():
            success, frame = cap.read()
            if not success:
                print("Ignoring empty camera frame.")
                break

            frame = cv2.flip(frame, 1)
            img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = hands.process(img_rgb)

            now = time.time()
            dt = max(now - last_time, 1e-3)
            last_time = now

            current_size_landmarks = None
            current_angle_landmarks = None

            if results.multi_hand_landmarks:
                hand_lms = results.multi_hand_landmarks[0]
                current_size_landmarks = hand_lms.landmark

                if results.multi_hand_world_landmarks:
                    current_angle_landmarks = results.multi_hand_world_landmarks[0].landmark
                else:
                    current_angle_landmarks = current_size_landmarks

                mp_draw.draw_landmarks(frame, hand_lms, mp_hands.HAND_CONNECTIONS)
                draw_middle_axis_overlay(frame, current_size_landmarks)

                if (
                    ref_height is not None
                    and ref_width is not None
                    and ref_roll is not None
                ):
                    error_x, error_y, error_a = compute_errors(
                        current_size_landmarks,
                        current_angle_landmarks,
                        ref_height,
                        ref_width,
                        ref_roll,
                    )

                    error_x = apply_deadzone(error_x, DEADZONE_X)
                    error_y = apply_deadzone(error_y, DEADZONE_Y)
                    error_a = apply_deadzone(error_a, DEADZONE_A)

                    velocity_x = pid_x.calculate(error_x, dt)
                    velocity_y = pid_y.calculate(error_y, dt)
                    velocity_a = pid_a.calculate(error_a, dt)

                    cv2.putText(
                        frame,
                        f"Err: x={error_x:+.2f} y={error_y:+.2f} a={error_a:+.2f}",
                        (10, 70),
                        cv2.FONT_HERSHEY_PLAIN,
                        1.6,
                        (255, 255, 0),
                        2,
                    )
                    cv2.putText(
                        frame,
                        f"Vel: x={velocity_x:+.2f} y={velocity_y:+.2f} a={velocity_a:+.2f}",
                        (10, 95),
                        cv2.FONT_HERSHEY_PLAIN,
                        1.6,
                        (0, 255, 255),
                        2,
                    )

                    print(
                        f"Errors -> X:{error_x:+.3f} Y:{error_y:+.3f} A:{error_a:+.3f} | "
                        f"Cmd -> Vx:{velocity_x:+.3f} Vy:{velocity_y:+.3f} Va:{velocity_a:+.3f}"
                    )
                else:
                    cv2.putText(
                        frame,
                        "Show hand and press 's' to calibrate",
                        (10, 70),
                        cv2.FONT_HERSHEY_PLAIN,
                        1.6,
                        (0, 0, 255),
                        2,
                    )
            else:
                cv2.putText(
                    frame,
                    "No hand detected",
                    (10, 40),
                    cv2.FONT_HERSHEY_PLAIN,
                    2,
                    (0, 0, 255),
                    2,
                )

            cv2.imshow("Hand Tracking for Omni-Bot", frame)

            key = cv2.waitKey(1) & 0xFF
            if (
                key == ord("s")
                and current_size_landmarks is not None
                and current_angle_landmarks is not None
            ):
                ref_height, ref_width, ref_roll = calibrate_hand(
                    current_size_landmarks, current_angle_landmarks
                )
                print(
                    f"Calibration saved -> height:{ref_height:.4f}, "
                    f"width:{ref_width:.4f}, roll(rad):{ref_roll:.4f}"
                )
            elif key == ord("q"):
                break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()