// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class StartEndCommandOnDisable extends StartEndCommand {
  /** Creates a new StartEndCommandOnDisable. */
  public StartEndCommandOnDisable(Runnable onInit, Runnable onEnd, Subsystem... requierments) {
    super(onInit, onEnd, requierments);
  }

  @Override
  public boolean runsWhenDisabled() {
      return true;
  }
}
