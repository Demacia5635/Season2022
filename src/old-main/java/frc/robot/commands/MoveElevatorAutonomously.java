// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ElivatorInside;
import frc.robot.utils.PID;

public class MoveElevatorAutonomously extends CommandBase {
  /** Creates a new MoveElevatorAutonomously. */

  private final ElivatorInside elivator;
  private double distance, target, current, power;
  private final PID powerPid;
  public MoveElevatorAutonomously(ElivatorInside elivator, double distance) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elivator = elivator;
    this.distance = distance;

    //set pid values
    powerPid = new PID(Constants.CLIMB_KP, Constants.CLIMB_KI, Constants.CLIMB_KD);

    addRequirements(elivator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    current = elivator.getSelectedSensorPositionInMeters();
    target = current + distance;
    powerPid.setPoint(target);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    power = powerPid.calculate(elivator.getSelectedSensorPositionInMeters());
    elivator.setPowerTelescopicMotor(power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elivator.setPowerTelescopicMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //stops close to the target
    if(target > elivator.getSelectedSensorPositionInMeters() - 0.02 &&
       target < elivator.getSelectedSensorPositionInMeters() + 0.02 )
       return true;

    return false;
  }
}
