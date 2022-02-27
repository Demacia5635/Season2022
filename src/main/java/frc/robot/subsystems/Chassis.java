// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.GroupOfMotors;

public class Chassis extends SubsystemBase{
  /** Creates a new Chassis. */
  private final GroupOfMotors left;
  private final GroupOfMotors right;
  private final SimpleMotorFeedforward aff;

  public Chassis(){
    left = new GroupOfMotors(Constants.LEFT_FRONT_PORT, Constants.LEFT_BACK_PORT);
    right = new GroupOfMotors(Constants.RIGHT_FRONT_PORT, Constants.RIGHT_BACK_PORT);
    aff = new SimpleMotorFeedforward(Constants.KS, Constants.KV);
  }

  public void setPower(double lPower, double rPower){
    left.setPower(lPower);
    right.setPower(rPower);
  }

  public void setAngularVelocity(double velocity, double turns){
    ChassisSpeeds speeds = new ChassisSpeeds(velocity * Constants.MAX_VELOCITY, 0, turns * Constants.MAX_ANGULAR_VELOCITY);

    DifferentialDriveWheelSpeeds wheelSpeeds = Constants.KINEMATICS.toWheelSpeeds(speeds);
    left.setVelocity(wheelSpeeds.leftMetersPerSecond, aff);
    right.setVelocity(wheelSpeeds.rightMetersPerSecond, aff);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
