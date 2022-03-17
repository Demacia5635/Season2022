// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.LedHandler;

public class MoveBetweenColors extends CommandBase {
  /** Creates a new MoveBetweenColors. */
  private final double startH, endH;
  private double currentH;
  private int isUp;
  private final LedHandler ledHandler;

  public MoveBetweenColors(double startH, double endH, LedHandler ledHandler) {
    this.startH = startH;
    this.endH = endH;
    this.ledHandler = ledHandler;
    addRequirements(ledHandler);
  }

  @Override
  public void initialize() {
    currentH = startH;
    isUp = 1;
  }

  @Override
  public void execute() {
    ledHandler.setColorWithOffsetAndRoofs(currentH, 255, 230, 1, endH, startH);
    currentH += isUp;
    if (currentH + Constants.LED_COUNT < startH || currentH > endH) {
      isUp *= -1;
    }
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
