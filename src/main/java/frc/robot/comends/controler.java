// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.comends;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

public class controler extends CommandBase {
  /** Creates a new controler. */
  private Chassis chassis;
  private XboxController controller;
  public controler(Chassis chassis, XboxController controller) {
    this.chassis = chassis;
    this.controller = controller;
    addRequirements(chassis);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
    //xboxcontroler wey with inputY
    chassis.leftpower(1/**inputX*/);

    //xboxcontroler wey with inputx
    chassis.rightpower(1/**inputX*/);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassis.leftpower(0);
    chassis.rightpower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
