// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Chassis;

public class MoveForward extends CommandBase {
  
  private final Chassis chassis;
  private final double meters;
  private double startingPosition;

  public MoveForward(Chassis chassis, double meters) {
    this.chassis = chassis;
    this.meters = meters;
    addRequirements(chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startingPosition = chassis.getAverageEncoderDistance();
    chassis.setPower(Constants.MOVE_POWER, Constants.MOVE_POWER);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassis.setPower(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return chassis.getAverageEncoderDistance() > startingPosition + meters;
  }
}
