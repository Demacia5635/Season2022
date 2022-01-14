// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    //Shooting
    public static final int SHOOTER_PORT = -1;
    public static final double SHOOTER_PULSE_TO_METER = -1;
    public static final double SHOOTER_KS = -1;
    public static final double SHOOTER_KV = -1;
    public static final int CAMERA_WIDTH_PIXELS = 640;
    public static final int CAMERA_HEIGHT_PIXELS = 480;
    public static final double MAX_SHOOT_ANGLE_ERROR = -1;
    public static final double MAX_SHOOT_VELOCITY_ERROR = -1;

    public static final double[] SHOOTING_VELOCITIES = {
        -1
    };
    public static final double SHOOTING_VELOCITIES_DIFF = -1;
    public static final double MIN_SHOOTING_DISTANCE = -1;

    //vision
    public static final int MIN_GRAYSCALE_VISION = 255;
    public static final int MAX_GRAYSCALE_VISION = 255;
    public static final int KERNEL_SIZE = 5;
    public static final double MIN_CONTOUR_SIZE = 20;
    public static final double TOWER_HEIGHT = 2.64;
    public static final double CAMERA_HEIGHT = -1;
    public static final int CAMERA_FOV_HEIGHT = -1;
    public static final double CAMERA_ANGLE = -1;
    public static final double CAMERA_FOV_WIDTH = 50;
    public static final double TOWER_RADIUS = 0.61;
}
