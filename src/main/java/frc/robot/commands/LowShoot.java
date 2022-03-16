// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooting;

public class LowShoot extends CommandBase {
  /** Creates a new LowShoot. */
  private final Shooting shooting;
  private static final double SHOOTING_VELOCITY = 6;
  public LowShoot(Shooting shooting) {
    this.shooting = shooting;
    addRequirements(shooting);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooting.setShooterVelocity(SHOOTING_VELOCITY);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shooting.getShooterVelocity2() >= SHOOTING_VELOCITY - 1) {
      shooting.openShooterInput();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooting.setShooterPower(0);
    shooting.closeShooterInput();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
