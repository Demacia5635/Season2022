// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pickup extends SubsystemBase {

  private final WPI_TalonSRX intake, arm;
  private final ArmFeedforward armFeedforward;

  /** Creates a new BallIntake. */
  public Pickup() {
    intake = new WPI_TalonSRX(Constants.INTAKE_PORT);
    arm = new WPI_TalonSRX(Constants.ARM_PORT);

    armFeedforward = new ArmFeedforward(Constants.KS, Constants.KCOS, Constants.KV);
    
    arm.setSelectedSensorPosition(Constants.PULSES_AT_THE_TOP);
  }

  public void setPower(double power){
    intake.set(ControlMode.PercentOutput, power);
  }
  public void setVelocity(double velocity){
    arm.set(ControlMode.PercentOutput, armFeedforward.calculate(degreesToRadians(getDegreeOfArm()), velocity));
  }
  public double degreesToRadians(double degree){
    return degree * (Math.PI / 180);
  }

  public double getArmSelectedSensorPosition(){
    return arm.getSelectedSensorPosition();
  }

  public double getDegreeOfArm(){
    return getArmSelectedSensorPosition() * ( 360 / Constants.ENCODER_PULSES);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
