// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.security.PublicKey;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    //Ports
    public static final int INTAKE_PORT = -1;
    public static final int ARM_PORT = -1;
    
    //Power
    public static final double PICKUP_POWER = -1;

    //Velocity
    public static final double ARM_UP_Velocity = -1;
    public static final double ARM_DOWN_Velocity = -1;

    //Encoder
    public static final double ENCODER_PULSES = -1;

    //Angles
    public static final double TOP_ARM_ANGLE = -1;
    public static final double BOTTOM_ARM_ANGLE = -1;

    //Positions
    public static final double PULSES_AT_THE_BOTTOM = BOTTOM_ARM_ANGLE  * (ENCODER_PULSES / 360); // converts angle to pulses
    public static final double PULSES_AT_THE_TOP = TOP_ARM_ANGLE * (ENCODER_PULSES / 360); // converts angle to pulses
    
    //Arm FeedForward
    public static final double KS = -1;
    public static final double KCOS = -1;
    public static final double KV = -1;

}
