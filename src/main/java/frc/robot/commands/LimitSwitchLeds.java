// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooting;
import frc.robot.subsystems.LedHandler;
import frc.robot.subsystems.Pickup;

public class LimitSwitchLeds extends CommandBase {
  /** Creates a new LimitSwitchLeds. */
  private final LedHandler ledHandler;
  private final Pickup pickup;
  private final Shooting shooting;

  public LimitSwitchLeds(LedHandler ledHandler, Pickup pickup, Shooting shooting) {
    this.ledHandler = ledHandler;
    this.pickup = pickup;
    this.shooting = shooting;
    addRequirements(ledHandler);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (pickup.getLowerLimit()) {
      ledHandler.setRangeColor(0, 5, 0, 255, 0);
    } else {
      ledHandler.setRangeColor(0, 5, 168, 0, 230);
    }

    if (pickup.getUpperLimit()) {
      ledHandler.setRangeColor(5, 10, 0, 255, 0);
    } else {
      ledHandler.setRangeColor(5, 10, 168, 0, 230);
    }

    if (shooting.getLowerLimitSwitch()) {
      ledHandler.setRangeColor(10, 15, 0, 255, 0);
    } else {
      ledHandler.setRangeColor(10, 15, 168, 0, 230);
    }

    if (shooting.getUpperLimitSwitch()) {
      ledHandler.setRangeColor(15, 20, 0, 255, 0);
    } else {
      ledHandler.setRangeColor(15, 20, 168, 0, 230);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

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
