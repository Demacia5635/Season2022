// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.LedHandler;

public class Rainbow extends CommandBase {
  /** Creates a new Rainbow. */
  private double currentH = 0;
  public Rainbow() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentH = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    LedHandler.setColorWithOffset(currentH, 255, 128, 5);
    currentH += 3;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    LedHandler.setColor(135, 45, 255);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
