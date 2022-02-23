// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Pickup;

public class SetArm extends CommandBase {
  /** Creates a new SetArm. */
  
  private final Pickup pickup;
  private final Destination destination;

  public enum Destination {
    UP, DOWN
  }

  public SetArm(Pickup pickup, Destination destination) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.pickup = pickup;
    this.destination = destination;
    
    addRequirements(pickup);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pickup.setArmPower(destination == Destination.UP ? Constants.ARM_UP_POWER : Constants.ARM_DOWN_POWER);
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pickup.setPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return destination == Destination.DOWN ? pickup.isDown() : !pickup.isDown();
  }
}
