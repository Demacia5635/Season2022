// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.drive.Vector2d;

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
    public static final int TURNER_PORT = -1;
    public static final int TURNER_GYRO_PORT = -1;
    public static final double SHOOTER_PULSE_TO_METER = -1;
    public static final double SHOOTER_KS = -1;
    public static final double SHOOTER_KV = -1;
    public static final double TURNER_DEFAULT_POWER = -1;
    public static final double MAX_SHOOT_ANGLE_ERROR = -1;
    public static final double MAX_SHOOT_VELOCITY_ERROR = -1;

    public static final Vector2d[] SHOOTING_VALUES = null;
    public static final double SHOOTING_VELOCITIES_DIFF = -1;
    public static final double MIN_SHOOTING_Y = -1;
}
