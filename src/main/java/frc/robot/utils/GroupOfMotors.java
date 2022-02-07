// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.Arrays;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Constants;

/** Add your docs here. */
public class GroupOfMotors {

    private final TalonFX lead;
    private final TalonFX[] motors;

    /**
     * creates a new GroupOfMotors where the first one has an encoder
     * @param motors 
     */
    public GroupOfMotors(TalonFX... motors){
        lead = motors[0];
        this.motors = motors;
        for (TalonFX motor : motors){
            if (motor == lead) continue;
            motor.follow(lead);
        }
    }

    /**
     * creates a new GroupOfMotors where the first one has an encoder
     * @param ports the ports for the motors 
     */
    public GroupOfMotors(int... ports){
        this(Arrays.stream(ports).mapToObj(port -> new TalonFX(port))
                .toArray(TalonFX[]::new));
    }

    public void setPower(double power) { // -1 <= power <= 1
        lead.set(ControlMode.PercentOutput, power);
    }

    public void setRelVelocity(double vel, SimpleMotorFeedforward aff) { // -1 <= vel <= 1
        setVelocity(vel * Constants.MAX_VELOCITY, aff);
    }

    public double getVelocity() {
        return lead.getSelectedSensorVelocity() / Constants.PULSES_PER_METER * 10;
    }

    public void resetEncoder() {
        lead.setSelectedSensorPosition(0);
    }

    public void setVelocity(double vel, SimpleMotorFeedforward aff) {// M/S
        double a = (vel - getVelocity());
        double af = aff.calculate(vel, a) / 12.;
        setVelocity(vel, af);
    }

    public void setVelocity(double vel, double aff) {
        lead.set(ControlMode.Velocity, vel * Constants.PULSES_PER_METER / 10.,
        DemandType.ArbitraryFeedForward, aff);
    }

    public void setPosition(double pos) {
        lead.set(ControlMode.Position, pos * Constants.PULSES_PER_METER);
    }

    public void setPosition(double pos, double ff) {
        lead.set(ControlMode.Position, pos * Constants.PULSES_PER_METER ,
                DemandType.ArbitraryFeedForward, ff);
    }

    public double getPower() {
        return lead.getMotorOutputPercent();         
    }
    /**
     * 
     * @param error - in sensor units
     */
    public void setClosedLoopAllowedError(double error) {
        lead.configAllowableClosedloopError(0, 150); 
    }

    public double getError() {
        return lead.getClosedLoopError(); 
    }
    public void setMotionMagic(double pos, SimpleMotorFeedforward aff, double maxSpeed,
            double acceleration) {
        // this.lead.set(ControlMode.MotionMagic, pos, DemandType.ArbitraryFeedForward,
        // aff.calculate(maxSpeed, acceleration));
        lead.set(ControlMode.MotionMagic, pos, DemandType.Neutral,
                aff.calculate(maxSpeed, acceleration));
    }

    public void setMotionMagic(double pos) {
        // this.lead.set(ControlMode.MotionMagic, pos, DemandType.Neutral, 0);
        lead.set(ControlMode.MotionMagic, pos, DemandType.AuxPID, 0);
    }

    public double getAccelForSpeed(double vel) {
        return vel - getVelocity();
    }

    public void setK_P(double k_p) {
        lead.config_kP(0, k_p);
    }

    public void setK_I(double k_i) {
        lead.config_kI(0, k_i);
    }

    public void setK_D(double k_d) {
        lead.config_kD(0, k_d);
    }

    public void invertMotors(boolean isInverted) {
        lead.setInverted(isInverted);
        for (TalonFX motor : motors) {
            motor.setInverted(InvertType.FollowMaster);
        }
    }

    /**
     * 
     * @return the encoder pulse count
     */
    public double getEncoder() { // Pulses
        return lead.getSelectedSensorPosition();
    }

    /**
     * 
     * @return the encoder count translated to meters
     */
    public double getDistance() { // Meters
        return getEncoder() / Constants.PULSES_PER_METER;
    }

    public void invertEncoder(boolean isInverted) {
        lead.setSensorPhase(isInverted);
    }

    /**
     * 
     * @param isBrake bake mode or coast
     * Sets the motors neutral mode to either brake or coast
     */
    public void setNeutralMode(boolean isBrake) {
        NeutralMode mode = isBrake ? NeutralMode.Brake : NeutralMode.Coast;
        lead.setNeutralMode(mode);
        for (TalonFX motor : motors) {
            motor.setNeutralMode(mode);
        }
    }

    /**
     * If the S-Curve strength [0,8] is set to a nonzero value,
     * the generated velocity profile is no longer trapezoidal,
     * but instead is continuous (corner points are smoothed).
     * The S-Curve feature, by its nature, will increase the amount of time a
     * movement requires.
     * This can be compensated for by decreasing the configured acceleration value.
     * 
     * @param curveStrength - the S-Curve strength between 0-8
     */
    public void setMotionSCurve(int curveStrength) {
        lead.configMotionSCurveStrength(curveStrength);
    }

    /**
     * configs the talon's cruise (peak) velocity
     * 
     * @param cruiseVelocity - in sensor units per 100ms
     */
    public void setCruiseVelocity(double cruiseVelocity) {
        lead.configMotionCruiseVelocity(cruiseVelocity);
    }

    /**
     * configs the motion acceleration
     * 
     * @param motionAcceleration - in sensor units per 100ms
     */
    public void setAcceleration(double motionAcceleration) {
        lead.configMotionAcceleration(motionAcceleration);
    }
}
