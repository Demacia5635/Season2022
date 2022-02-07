// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ElivatorInside;

public class MoveElivator extends CommandBase {
  /** Creates a new MoveElivator. */
  private final ElivatorInside elivator;
  private final XboxController controller;

  public MoveElivator(ElivatorInside eInside, XboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elivator = eInside;
    this.controller = controller;
    addRequirements(elivator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double power = deadband(-controller.getRightY());

    elivator.setPowerTelescopicMotor(power);
  }

  /**
   * deadbands the value of the controller
   * @param value between (-1 and 1)
   * @return 0 if under the minimum, the value otherwise
   */
  private double deadband(double value){
    return (Math.abs(value) < Constants.JOYSTICK_DEADBAND ? 0 : value);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elivator.setPowerTelescopicMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
