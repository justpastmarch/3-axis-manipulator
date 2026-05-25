# pixel_to_robot_simple.py

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480

BOARD_WIDTH_MM = 400.0
BOARD_HEIGHT_MM = 300.0

ROBOT_BASE_X = 200.0
ROBOT_BASE_Y = 0.0

FLIP_Y = True


def pixel_to_robot(cx, cy):

    # pixel -> board(mm)
    x_board = (cx / CAMERA_WIDTH) * BOARD_WIDTH_MM

    if FLIP_Y:
        y_board = BOARD_HEIGHT_MM - (
            (cy / CAMERA_HEIGHT) * BOARD_HEIGHT_MM
        )
    else:
        y_board = (cy / CAMERA_HEIGHT) * BOARD_HEIGHT_MM

    # board -> robot
    x_robot = x_board - ROBOT_BASE_X
    y_robot = y_board - ROBOT_BASE_Y

    return x_robot, y_robot


if __name__ == "__main__":

    cx = 340
    cy = 220

    x_robot, y_robot = pixel_to_robot(cx, cy)

    print(f"pixel: ({cx}, {cy})")
    print(f"robot: ({x_robot:.2f} mm, {y_robot:.2f} mm)")