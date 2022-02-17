// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooting;

public class ShootingCalibration extends CommandBase {
  
  private double velocity;
  private double angle;
  private final Shoot shootCommand;
  private double[] velocities;
  private double[] angles;

  public ShootingCalibration(Shooting shooting) {
    shootCommand = new Shoot(shooting, () -> {return velocity;}, () -> {return 0;});
    velocities = new double[0];
    angles = new double[0];
    velocity = 0;
    angle = 0;
  }

  @Override
  public void end(boolean interrupted) {
    shootCommand.cancel();
    for (int i = 0; i < angles.length; i++){
      System.out.println("Angle" + i + ": " + angles[i]);
      System.out.println("Velocity" + i + ": " + velocities[i]);
      System.out.println("--------------------------------------------------------");
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  public void start(boolean toStart){
    if (toStart){
      schedule();
    }
    else {
      cancel();
    }
  }

  public void setVelocity(double velocity){
    this.velocity = velocity;
  }

  public void setAngle(double angle){
    this.angle = angle;
  }

  public void shoot(boolean toShoot){
    if (toShoot){
      shootCommand.schedule();
    }
    else {
      shootCommand.cancel();
    }
  }

  public void save(){
    double[] temp = new double[velocities.length + 1];
    double[] temp2 = new double[angles.length + 1];

    for (int i = 0; i < velocities.length; i++){
      temp[i] = velocities[i];
      temp2[i] = angles[i];
    }
    temp[velocities.length] = velocity;
    temp2[angles.length] = angle;
    velocities = temp;
    angles = temp2;
  }
}
