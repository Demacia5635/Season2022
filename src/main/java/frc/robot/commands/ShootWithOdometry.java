// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.LedHandler;
import frc.robot.subsystems.Shooting;

public class ShootWithOdometry extends CommandBase {

  private final Shooting shooting;
  private final LedHandler ledHandler;
  private final Chassis chassis;
  private int time;

  public ShootWithOdometry(Shooting shooting, LedHandler ledHandler, Chassis chassis) {
    this.shooting = shooting;
    this.ledHandler = ledHandler;
    this.chassis = chassis;
    addRequirements(shooting, ledHandler, chassis);
  }

  @Override
  public void initialize() {
    chassis.setNeutralMode(true);
    ledHandler.setColor(255, 255, 0);
    time = 0;
  }

  @Override
  public void execute() {
    double wantedVelocity = shooting.getShootingVelocity(chassis.getDistanceToHub());
    double velocity = shooting.getShooterVelocity2();
    shooting.setShooterVelocity(wantedVelocity);

    if (velocity >= wantedVelocity - 1) {
      shooting.openShooterInput();
    }
    shooting.setTurnerPower(-0.5);
    if (velocity >= wantedVelocity - 1 || time > 0) {
      time++;
    }
    if (time < 10)
      ledHandler.setRangeColor((int) (Constants.LED_COUNT / 2 - (Math.min((velocity / wantedVelocity * Constants.LED_COUNT / 2),
          Constants.LED_COUNT / 2))), Constants.LED_COUNT / 2, 0, 255, 0);
    else
      ledHandler.setColor(255, 0, 0);

    double heading = chassis.getAngleToHub();
    velocity = heading * Constants.ANGLE_KP;
    velocity = Math.signum(velocity) * Math.max(Math.abs(velocity), 0.5);
    SmartDashboard.putNumber("Angle Velocity", velocity);
    if(Math.abs(heading) > Constants.MAX_ANGLE_ERROR_CHASSIS)
        chassis.setVelocity(-velocity, velocity);
    else chassis.setVelocity(0, 0);
  }

  @Override
  public void end(boolean interrupted) {
    shooting.setShooterPower(0);
    shooting.closeShooterInput();
    new StartEndCommand(() -> {
      shooting.setTurnerPower(0.5);
    }, () -> {
      shooting.setTurnerPower(0);
    }).withTimeout(0.15).schedule();
    chassis.setNeutralMode(false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
