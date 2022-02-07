// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Drive;
import frc.robot.commands.MoveElevatorAutonomously;
import frc.robot.commands.OpenShackle;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.ElivatorInside;
import frc.robot.subsystems.Pickup;
import frc.robot.subsystems.Shooting;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Pickup pickup;
  private final ElivatorInside elivator_Inside;
  private final Chassis chassis;
  private final Shooting shooting;
  
  private final XboxController secondaryController;
  private final XboxController mainController;
  
  private final JoystickButton aButton;
  private final JoystickButton step1Button;
  private final JoystickButton step2Button;
  private final JoystickButton trigerForShackle;
  
  private final OpenShackle openShackle;
  private final MoveElevatorAutonomously step1;
  private final MoveElevatorAutonomously step2;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */

  public RobotContainer() {
    secondaryController = new XboxController(1);
    mainController = new XboxController(0);

    chassis = new Chassis();
    pickup = new Pickup();
    elivator_Inside = new ElivatorInside(secondaryController);
    shooting = new Shooting();

    trigerForShackle = new JoystickButton(secondaryController, Constants.TRIGER_FOR_SHACKLE);
    aButton = new JoystickButton(mainController, 1);
    step1Button = new JoystickButton(secondaryController, Constants.STEP_1_BUTTON);
    step2Button = new JoystickButton(secondaryController, Constants.STEP_2_BUTTON);
    
    openShackle = new OpenShackle(elivator_Inside);
    
    //Autonomous
    step1 = new MoveElevatorAutonomously(elivator_Inside, Constants.DISTANCE_STEP_1);
    step2 = new MoveElevatorAutonomously(elivator_Inside, Constants.DISTANCE_STEP_2);

    chassis.setDefaultCommand(new Drive(chassis, mainController));
    
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   * A button main -> pickup balls
   */
  private void configureButtonBindings() {
    trigerForShackle.whileHeld(openShackle);
    step1Button.whenPressed(step1);
    step2Button.whenPressed(step2);
    aButton.whenHeld(new StartEndCommand(() -> {pickup.setPower(Constants.PICKUP_POWER);},
        () -> {pickup.setPower(0);}, pickup));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
