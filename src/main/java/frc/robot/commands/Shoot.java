// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shooting;

public class Shoot extends CommandBase {
  
  private final Shooting shooting;
  private final DoubleSupplier velocityGetter;
  private final DoubleSupplier angleGetter;

  public Shoot(Shooting shooting, DoubleSupplier velocityGetter, DoubleSupplier angleGetter) {
    this.shooting = shooting;
    this.velocityGetter = velocityGetter;
    this.angleGetter = angleGetter;

    addRequirements(shooting);
  }

  public Shoot(Shooting shooting, double velocity, double angle) {
    this(shooting, () -> {return velocity;}, () -> {return angle;});
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    shooting.setShooterVelocity(velocityGetter.getAsDouble());
    shooting.setTurnerPower(Math.signum(angleGetter.getAsDouble() - shooting.getTurnerAngle()) * Constants.TURNER_DEFAULT_POWER);

    if (Math.abs(shooting.getShooterVelocity() - velocityGetter.getAsDouble()) <= Constants.MAX_SHOOT_VELOCITY_ERROR && 
        Math.abs(angleGetter.getAsDouble() - shooting.getTurnerAngle()) <= Constants.MAX_SHOOT_ANGLE_ERROR){
          
      shooting.openShooterInput();
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooting.closeShooterInput();
    shooting.setShooterPower(0);
    shooting.setTurnerPower(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
