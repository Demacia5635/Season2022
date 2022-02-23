// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

/** Add your docs here. */
public class FeedForward {
    private double kS;
    private double kV;
    private double kA;

    public FeedForward(double kS, double kV){
        this.kS = kS;
        this.kV = kV;
        kA = 0;
    }

    public FeedForward(double kS, double kV, double kA){
        this(kS, kV);
        this.kA = kA;
    }

    /**
     * calculates the estimated power needed to reach the velocity
     * @param velocity in meter/sec
     * @return the power, between 1 and -1
     */
    public double get(double velocity){
        return kS * Math.signum(velocity) + kV * velocity;
    }

    /**
     * calculates the estimated power needed to reach the velocity with the acceleration specified
     * @param velocity in meter/sec
     * @param accel in meter/sec^2
     * @return the power, between 1 and -1
     */
    public double get(double velocity, double accel){
        return kS * Math.signum(velocity) + kV * velocity + accel * kA;
    }
}
