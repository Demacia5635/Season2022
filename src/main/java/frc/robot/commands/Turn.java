// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Chassis;
import frc.robot.utils.PID;

public class Turn extends CommandBase {
  /** Creates a new Turn. */
  private Chassis chassis;
  private double angle;
  private PID pidAngle;
  private double startingDistance;
  private double currentAngle;
  private boolean toStop;
  public Turn(Chassis chassis, double angle, boolean toStop) {
    this.chassis = chassis;
    this.toStop = toStop;
    this.angle = angle;
    pidAngle = new PID(Constants.ANGLE_KP, Constants.ANGLE_KI, Constants.ANGLE_KD);
    addRequirements(chassis);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startingDistance = chassis.getAngle();
    pidAngle.setPoint(angle + startingDistance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentAngle = chassis.getAngle();
    double velocityRatio = pidAngle.calculate(currentAngle);
    chassis.setVelocity(velocityRatio, -velocityRatio);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassis.setPower(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(toStop == true)
      return chassis.getAngle() >= ((angle + startingDistance)-Constants.STOP_ANGLE);
    else{
      return toStop;
    }
  }
}
