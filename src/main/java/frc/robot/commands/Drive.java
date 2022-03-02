// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Chassis;

public class Drive extends CommandBase {

  private final Chassis chassis;
  private final XboxController controller;

  public Drive(Chassis chassis, XboxController controller) {
    this.chassis = chassis;
    this.controller = controller;

    addRequirements(chassis);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double value = deadband((chassis.isReversed() ? -1 : 1) * (controller.getRightTriggerAxis() - controller.getLeftTriggerAxis()));
    double angle = deadband(controller.getLeftX());

    chassis.setAngularVelocity(value, -angle);
  }

  private double deadband(double value){
    return Math.abs(value) > Constants.CONTROLLER_DEADBAND ? value : 0;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassis.setPower(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
