// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final double MAX_VELOCITY = 3;
    public static final double MAX_ANGULAR_VELOCITY = Math.PI / 2;
    public static final double PULSES_PER_METER = 12.0*2048/(6.0*25.4*Math.PI/1000.0);

    public static final int LEFT_FRONT_PORT = 3; 
    public static final int LEFT_BACK_PORT = 4; 
    public static final int RIGHT_FRONT_PORT = 1; 
    public static final int RIGHT_BACK_PORT = 2;
    public static final int GYRO_PORT = 5;
    public static final double CONTROLLER_DEADBAND = 0.1;
    public static final double KS = -1;
    public static final double KV = -1;
    public static final double KP = -1;
    public static final double KA = 0;
    public static final double TRACK_WIDTH = 0.56;
    public static final DifferentialDriveKinematics KINEMATICS = new DifferentialDriveKinematics(TRACK_WIDTH);

    public static final int ANGLE_KP = -1;
    public static final int ANGLE_KI = -1;
    public static final int ANGLE_KD = -1;

    public static final int STOP_ANGLE = 5;

    public static final double MAX_VELOCITY_AUTO = -1;
    public static final double MAX_ACCELERATION_AUTO = -1;

    public static final double MOVE_POWER = 0.6;
    public static final double TURN_POWER = 0.6;

    public static final double RAMSETE_B = 2;
    public static final double RAMSETE_ZETA = 0.7;
}
