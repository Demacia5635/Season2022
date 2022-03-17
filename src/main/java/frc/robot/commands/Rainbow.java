// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LedHandler;

public class Rainbow extends CommandBase {
  /** Creates a new Rainbow. */
  private double currentH = 0;
  private final LedHandler ledHandler;

  public Rainbow(LedHandler ledHandler) {
    this.ledHandler = ledHandler;
    addRequirements(ledHandler);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentH = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ledHandler.setColorWithOffset(currentH, 255, 128, 5);
    currentH += 3;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ledHandler.setDefaultColor();
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
