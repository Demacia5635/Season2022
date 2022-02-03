// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.GroupOfMotors;

public class Chassis extends SubsystemBase{
  /** Creates a new Chassis. */
  private final GroupOfMotors leftMotors;
  private final GroupOfMotors rightMotors;

  public Chassis(){
    leftMotors = new GroupOfMotors(Constants.LEFT_FRONT_PORT, Constants.LEFT_BACK_PORT);
    rightMotors = new GroupOfMotors(Constants.RIGHT_FRONT_PORT, Constants.RIGHT_BACK_PORT);
  }

  public void setPower(double lPower, double rPower){
    leftMotors.setPower(lPower);
    rightMotors.setPower(rPower);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
