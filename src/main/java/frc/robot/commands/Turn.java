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
  public Turn(Chassis chassis, double angle) {
    this.chassis = chassis;
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
    double pPower = pidAngle.calculate(currentAngle);
    chassis.setPower(pPower, -pPower);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
