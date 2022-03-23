// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LedHandler;

public class Rainbow extends CommandBase {
  /** Creates a new Rainbow. */
  private double rainbowSpeed = 1.;
  private double currentH = 0;
  private final LedHandler ledHandler;

  public Rainbow(LedHandler ledHandler) {
    this.ledHandler = ledHandler;
    addRequirements(ledHandler);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("Leds/Rainbow Speed", rainbowSpeed);
    currentH = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rainbowSpeed = SmartDashboard.getNumber("Leds/Rainbow Speed", 1.0);
    ledHandler.setColorWithOffset(currentH, 255, 128, 5 * rainbowSpeed);
    currentH += 3 * rainbowSpeed;
    currentH %= 180;
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
