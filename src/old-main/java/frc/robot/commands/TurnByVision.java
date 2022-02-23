// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Chassis;

public class TurnByVision extends CommandBase {
  /** Creates a new TurnByViision. */
  private Chassis chassis;
  private DoubleSupplier x;
  public TurnByVision(Chassis chassis, DoubleSupplier x) {
    this.chassis = chassis;
    this.x = x;
    addRequirements(chassis);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double power = Math.signum(x.getAsDouble())*Constants.TURN_POWER;
    chassis.setPower(-power, power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(x.getAsDouble()) < Constants.MAX_ANGLE_ERROR_CHASSIS;
  }
}
