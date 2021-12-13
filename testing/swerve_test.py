import math
from typing import Union

import cv2

from oi.keymap import Keymap
from swerve_sim_subsystem import drivetrain, drive_command, TestSwerveNode

import numpy as np

from utils import logger
from utils.math import rotate_vector

import pygame

pygame.init()
pygame.joystick.init()
joystick_init = False

img = np.zeros((600, 800, 3))

TW_PIXELS = 75

robot_x = 2
robot_y = 4

motor_vel_scale = 0.2


def p(*x: float) -> Union[int, tuple[int, ...]]:
    if len(x) == 1:
        return int(x[0] * TW_PIXELS)
    return tuple(int(i * TW_PIXELS) for i in x)


def c(x: float, y: float):
    return x, 600 - y


def draw_robot(x: float, y: float, theta: float):
    pts = [
        rotate_vector(-0.5, -0.5, theta),
        rotate_vector(+0.5, -0.5, theta),
        rotate_vector(+0.5, +0.5, theta),
        rotate_vector(-0.5, +0.5, theta)
    ]

    def draw_motor(m: TestSwerveNode, px: float, py: float, theta_off: float):
        x1, y1 = p(x + px, y + py)
        dx, dy = p(*rotate_vector(
            m.get_current_velocity() * motor_vel_scale,
            0,
            m.get_current_angle_raw() + theta_off + theta
        ))
        x2, y2 = x1 + dx, y1 + dy
        cv2.line(img, c(x1, y1), c(x2, y2), (0, 0, 255) if not m.motor_reversed else (255, 0, 0), 3)

    p_pts = []
    for a, b in pts:
        p_pts.append(c(int(p(a + x)), int(p(b + y))))

    cv2.fillPoly(img, [np.array(p_pts)], (128, 128, 0))

    draw_motor(drivetrain.n_00, *pts[0], 0)
    draw_motor(drivetrain.n_10, *pts[1], math.pi)
    draw_motor(drivetrain.n_01, *pts[3], math.pi)
    draw_motor(drivetrain.n_11, *pts[2], 0)


time_scale = 0.5
frame_scale = 1

x = 0

while cv2.waitKey(int(20 * frame_scale)) != ord("q"):
    drive_command.execute()
    drivetrain.n_00.update(0.02 * time_scale)
    drivetrain.n_01.update(0.02 * time_scale)
    drivetrain.n_10.update(0.02 * time_scale)
    drivetrain.n_11.update(0.02 * time_scale)
    img.fill(0)
    # drivetrain.axis_dx.val = math.sin(x)
    # drivetrain.axis_dy.val = math.cos(x)
    # drivetrain.axis_rotation.val = math.sin(x)
    x += 0.1 * time_scale
    try:
        pygame.event.pump()
        joystick = pygame.joystick.Joystick(0)
        if joystick_init:
            joystick.init()
            joystick_init = False
        drivetrain.axis_dx.val = joystick.get_axis(0)
        drivetrain.axis_dy.val = -joystick.get_axis(1)
        drivetrain.axis_rotation.val = -joystick.get_axis(3)
    except pygame.error:
        joystick_init = False
    drivetrain.odometry.angle_radians += (drivetrain.axis_rotation.val / (4 * math.pi / 3)) * time_scale * frame_scale
    robot_x += (drivetrain.axis_dx.val / 3) * time_scale * frame_scale
    robot_y += (drivetrain.axis_dy.val / 3) * time_scale * frame_scale
    draw_robot(robot_x, robot_y, drivetrain.odometry.angle_radians)
    cv2.imshow("swerve", img)
