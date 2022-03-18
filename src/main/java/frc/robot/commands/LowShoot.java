// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LedHandler;
import frc.robot.subsystems.Shooting;

public class LowShoot extends CommandBase {

  private final Shooting shooting;
  private final LedHandler ledHandler;
  private static final double SHOOTING_VELOCITY = 8;

  public LowShoot(Shooting shooting, LedHandler ledHandler) {
    this.shooting = shooting;
    this.ledHandler = ledHandler;
    addRequirements(shooting, ledHandler);
  }

  @Override
  public void initialize() {
    shooting.setShooterVelocity(SHOOTING_VELOCITY);
    ledHandler.setColor(255, 255, 0);
  }

  @Override
  public void execute() {
    double vel = shooting.getShooterVelocity2();
    if (vel >= SHOOTING_VELOCITY - 1) {
      shooting.openShooterInput();
    }
    if (!shooting.getUpperLimitSwitch()) {
      shooting.setTurnerPower(-0.3);
    } else {
      shooting.setTurnerPower(0);
    }
    ledHandler.setColor((int) (vel / SHOOTING_VELOCITY * 20), 0, 255, 0);
    if (vel >= SHOOTING_VELOCITY) {
      ledHandler.setColor(255, 0, 0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooting.setShooterPower(0);
    shooting.closeShooterInput();
    shooting.setTurnerPower(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
