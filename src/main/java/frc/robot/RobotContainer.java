// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.Drive;
import frc.robot.commands.OpenShackle;
import frc.robot.commands.SetArm;
import frc.robot.commands.SetArm.Destination;
import frc.robot.commands.Shoot;
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
  private final ElivatorInside elivatorInside;
  private final Chassis chassis;
  private final Shooting shooting;
  
  private final XboxController secondaryController;
  private final XboxController mainController;
  
  private final JoystickButton aButtonMain;
  private final JoystickButton yButtonMain;
  private final JoystickButton bButtonMain;
  private final JoystickButton rightBumperMain;
  private final JoystickButton startButtonMain;

  private final JoystickButton bButtonSecondary;
  
  private final OpenShackle openShackle;
  private final AutoShoot autoShoot;
  private final SetArm armUp;
  private final Command intake;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */

  public RobotContainer() {
    secondaryController = new XboxController(1);
    mainController = new XboxController(0);

    chassis = new Chassis();
    pickup = new Pickup();
    elivatorInside = new ElivatorInside(mainController);
    shooting = new Shooting();

    bButtonSecondary = new JoystickButton(secondaryController, 2);
    
    aButtonMain = new JoystickButton(mainController, 1);
    bButtonMain = new JoystickButton(mainController, 2);
    yButtonMain = new JoystickButton(mainController, 4);
    rightBumperMain = new JoystickButton(mainController, 6);
    startButtonMain = new JoystickButton(secondaryController, 8);

    openShackle = new OpenShackle(elivatorInside);
    autoShoot = new AutoShoot(shooting, chassis);
    armUp = new SetArm(pickup, Destination.UP);
    intake = new SetArm(pickup, Destination.DOWN).andThen(new StartEndCommand(
        () -> {pickup.setPower(Constants.PICKUP_POWER);},() -> {pickup.setPower(0);}, pickup));

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
   * Y buttom main -> arm up
   * B button main -> shoot
   * X button main -> vision tracking
   * right bumper main -> open shackle
   * B button secondary -> default shoot
   * start button secondary -> start climb sequence
   * right joystick y -> move elevator
   */
  private void configureButtonBindings() {
    aButtonMain.whenHeld(intake);
      
    yButtonMain.whenPressed(armUp);
    
    bButtonMain.whenHeld(autoShoot);

    rightBumperMain.whileHeld(openShackle);

    startButtonMain.whenPressed(new InstantCommand(() -> {elivatorInside.changeClimbingMode();}));

    bButtonSecondary.whileHeld(new Shoot(shooting, Constants.SHOOTING_DEFAULT_VELOCITY, Constants.SHOOTING_DEFAULT_ANGLE));
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
