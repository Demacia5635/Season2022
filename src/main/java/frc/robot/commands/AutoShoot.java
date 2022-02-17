// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Shooting;

public class AutoShoot extends CommandBase {
  
  private final Chassis chassis;
  private final Shooting shooting;
  private Command command;

  public AutoShoot(Shooting shooting, Chassis chassis) {
    this.chassis = chassis;
    this.shooting = shooting;
  }

  @Override
  public void initialize() {

    Shoot shootCommand = new Shoot(shooting, () -> {return calculateValues(shooting.getVisionDistance()).x;},
        () -> {return calculateValues(shooting.getVisionDistance()).y;});

    command = new TurnByVision(chassis, shooting::getVisionX).andThen(
        new InstantCommand(() -> {shootCommand.shoot();}).alongWith(shootCommand));
  }

  /**
   * calculates the velocity and the angle needed to shoot to the target
   * @param y the y to the tower, in pixels, usually from the vision
   * @return a Vector2d where x is the velocity in meter/sec and y is the angle if out of range, returns null
   */
  private Vector2d calculateValues(double y){
    if (y < Constants.MIN_SHOOTING_DISTANCE){
      cancel();
      return null;
    }

    for (int i = 0; i < Constants.SHOOTING_VALUES.length; i++) {
      double currentY = Constants.MIN_SHOOTING_DISTANCE + Constants.SHOOTING_VELOCITIES_DIFF * i;
      if (currentY > y){
        double slopeVel = (Constants.SHOOTING_VALUES[i].x - Constants.SHOOTING_VALUES[i - 1].x) /
            Constants.SHOOTING_VELOCITIES_DIFF;
        
        double slopeAngle = (Constants.SHOOTING_VALUES[i].y - Constants.SHOOTING_VALUES[i - 1].y) /
            Constants.SHOOTING_VELOCITIES_DIFF;

        return new Vector2d(slopeVel * (y - currentY) + Constants.SHOOTING_VALUES[i].x,
            slopeAngle * (y - currentY) + Constants.SHOOTING_VALUES[i].y);
      }
    }

    //means that the distance is above the maximum
    cancel();
    return null;
  }

  @Override
  public void end(boolean interrupted) {
    command.cancel();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
