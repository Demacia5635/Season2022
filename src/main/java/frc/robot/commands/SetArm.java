// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.CANifier.PinValues;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Pickup;

public class SetArm extends CommandBase {
  /** Creates a new SetArm. */
  
  private final Pickup pickup;
  private final Destination destination;
  private double target, velocity;

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
    switch(destination){
      
      case UP:
            target = Constants.PULSES_AT_THE_TOP;
            velocity = Constants.ARM_UP_Velocity;
            break;

      case DOWN:
                target = Constants.PULSES_AT_THE_BOTTOM;
                velocity = Constants.ARM_DOWN_Velocity;
                break;

    }
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pickup.setVelocity(velocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pickup.setVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(target > pickup.getArmSelectedSensorPosition() - 5 && target < pickup.getArmSelectedSensorPosition() + 5){
      return true;
    }
    return false;
  }
}
