// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
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
    //TODO : set constants
    //#region Shooting
    public static final int SHOOTER_PORT_MAIN = 5;
    public static final int SHOOTER_PORT_SECONDARY = 6;
    public static final int TURNER_PORT = 11;
    public static final int TURNER_GYRO_PORT = 13;
    public static final int INPUT_WHEEL_PORT = 9;
    public static final int LIMIT_SWITCH_PORT = 0;
    public static final double PULSE_TO_ANGLE = 360./4000;
    public static final double INPUT_WHEEL_POWER = -1;
    public static final double SHOOTER_PULSE_TO_METER = 0.1 * Math.PI / 2048;
    public static final double SHOOTER_KS = 0.04;
    public static final double SHOOTER_KV = 1 / 34.;
    public static final double SHOOTER_KP = 1 / 1000.;
    public static final double TURNER_DEFAULT_POWER = -0.4;
    public static final double MAX_SHOOT_ANGLE_ERROR = 1;
    public static final double MAX_SHOOT_VELOCITY_ERROR = 1.5;
    public static final double SHOOTING_DEFAULT_VELOCITY = 10;
    public static final double SHOOTING_DEFAULT_ANGLE = 45;
    public static final double SHOOTING_AUTO_VELOCITY = 20;
    public static final double SHOOTING_AUTO_ANGLE = 20;
    public static final double CAMERA_TOWER_DIFF = 1.5;
    public static final double MAX_ANGLE_ERROR_CHASSIS = 10;
    public static final double CAMERA_ANGLE = 0;

    public static final Vector2d[] SHOOTING_VALUES = {
        new Vector2d(11, 52)
    };
    public static final double SHOOTING_VELOCITIES_DIFF = -1;
    public static final double MIN_SHOOTING_DISTANCE = 1.3;
    //#endregion

    //#region Chassis
    public static final double MAX_VELOCITY = -1;
    public static final double PULSES_PER_METER = 1;

    public static final int LEFT_FRONT_PORT = 3; 
    public static final int LEFT_BACK_PORT = 4; 
    public static final int RIGHT_FRONT_PORT = 1; 
    public static final int RIGHT_BACK_PORT = 2; 

    public static final int GYRO_PORT = 14;

    public static final int ANGLE_KP = -1;
    public static final int ANGLE_KI = -1;
    public static final int ANGLE_KD = -1;

    public static final int STOP_ANGLE = 5;

    public static final double KS = -1;
    public static final double KV = -1;
    public static final double KA = -1;
    public static final double KP = -1;
    public static final double TRACK_WIDTH = 0.55;

    public static final DifferentialDriveKinematics KINEMATICS = new DifferentialDriveKinematics(TRACK_WIDTH);

    public static final double MAX_VELOCITY_AUTO = -1;
    public static final double MAX_ACCELERATION_AUTO = -1;

    public static final double MOVE_POWER = 0.3;
    public static final double TURN_POWER = 0.3;

    public static final double RAMSETE_B = 2;
    public static final double RAMSETE_ZETA = 0.7;
    //#endregion

    //#region Climb
    //motors
    public static final int TELESCOPIC_MOTOR = 12;
    public static final int SHACKLE_OPENNER = 8;
    
    //power
    public static final double SHACKLE_OPENNING_MAX_POWER = -0.5;

    //controller
    public static final double JOYSTICK_DEADBAND = 0.1;

    //elivator
    public static final double CLIMB_PULSES_PER_METER = -1;

    //distane to move
    public static final double DISTANCE_STEP_1 = -1;
    public static final double DISTANCE_STEP_2 = -1;

    //PID for climbing
    public static final double CLIMB_KP = -1;
    public static final double CLIMB_KI = -1;
    public static final double CLIMB_KD = -1;

    //FeedForward for climbing
    public static final double CLIMB_KS = -1;
    public static final double CLIMB_KV = -1;
    public static final double CONTROLLER_DEADBAND = 0.1;
    //#endregion
    
    //#region Intake
    //Ports
    public static final int INTAKE_PORT = 7;
    public static final int ARM_PORT = 10;
    
    //Power
    public static final double PICKUP_POWER = 1;

    //Velocity
    public static final double ARM_UP_POWER = 0.7;
    public static final double ARM_DOWN_POWER = -0.3;
    //#endregion
}
