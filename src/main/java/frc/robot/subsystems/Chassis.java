// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.GroupOfMotors;

public class Chassis extends SubsystemBase{
  /** Creates a new Chassis. */

  public void leftpower(double power) {// between 1 and -1
    new GroupOfMotors(Constants.LEFT_FRONT_PORT);
  }
  public void rightpower(double power) {// between 1 and -1
    new GroupOfMotors(Constants.RIGHT_FRONT_PORT);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
