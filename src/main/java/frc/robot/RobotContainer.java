// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AngleForLow;
import frc.robot.commands.Drive;
import frc.robot.commands.LimitSwitchLeds;
import frc.robot.commands.LowShoot;
import frc.robot.commands.MoveBetweenColors;
import frc.robot.commands.MoveForward;
import frc.robot.commands.MoveShackle;
import frc.robot.commands.Rainbow;
import frc.robot.commands.SetArm;
import frc.robot.commands.SetTurnerDown;
import frc.robot.commands.StartEndCommandOnDisable;
import frc.robot.commands.Turn;
import frc.robot.commands.SetArm.Destination;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.ElivatorInside;
import frc.robot.subsystems.LedHandler;
import frc.robot.subsystems.Pickup;
import frc.robot.subsystems.Shooting;
import frc.robot.utils.ShootingUtil;

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
  private final LedHandler ledHandler;
  
  private final XboxController secondaryController;
  private final XboxController mainController;
  
  private final JoystickButton aButtonMain;
  private final JoystickButton yButtonMain;
  private final JoystickButton xButtonMain;
  private final JoystickButton startButtonSecondary;
  private final JoystickButton backButtonMain;

  private final JoystickButton xButtonSecondary;
  private final JoystickButton bButtonSecondary;
  private final JoystickButton yButtonSecondary;
  private final JoystickButton aButtonSecondary;
  private final JoystickButton backButtonSecondary;
  
  private final MoveShackle openShackle;
  private final MoveShackle closeShackle;
//  private final AutoShoot autoShoot;
  private final Command intake;
  private final Command shoot;
  private final Command shoot2;
  private final Command throwOut;

  private final SendableChooser<Autonomouses> autonomousChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */

  public RobotContainer() {

    secondaryController = new XboxController(1);
    mainController = new XboxController(0);

    chassis = new Chassis();
    pickup = new Pickup();
    elivatorInside = new ElivatorInside(secondaryController);
    shooting = new Shooting(chassis);
    ledHandler = new LedHandler();
    SmartDashboard.putData("Limit Switch Leds", new LimitSwitchLeds(ledHandler, pickup, shooting));

    bButtonSecondary = new JoystickButton(secondaryController, 2);
    xButtonSecondary = new JoystickButton(secondaryController, 3);
    yButtonSecondary = new JoystickButton(secondaryController, 4);
    aButtonSecondary = new JoystickButton(secondaryController, 1);
    backButtonSecondary = new JoystickButton(secondaryController, 7);
    startButtonSecondary = new JoystickButton(secondaryController, 8);
    
    aButtonMain = new JoystickButton(mainController, 1);
    xButtonMain = new JoystickButton(mainController, 3);
    yButtonMain = new JoystickButton(mainController, 4);
    backButtonMain = new JoystickButton(mainController, 7);

    openShackle = new MoveShackle(elivatorInside, MoveShackle.Destination.OPEN);
    closeShackle = new MoveShackle(elivatorInside, MoveShackle.Destination.CLOSE);
//    autoShoot = new AutoShoot(shooting, chassis);
    intake = pickup.getIntakeCommand();
    shoot2 = new LowShoot(shooting, ledHandler).alongWith(pickup.getIntakeCommand());
    shoot = new StartEndCommand(() -> {chassis.setNeutralMode(true);}, () -> {chassis.setNeutralMode(false);}).alongWith(new MoveForward(chassis, 0.3).andThen(new LowShoot(shooting, ledHandler).alongWith(pickup.getIntakeCommand())));//new Shoot(shooting, chassis).alongWith(pickup.getIntakeCommand());
    throwOut = new StartEndCommand(() -> {
      shooting.setShooterVelocity(5);
      shooting.openShooterInput();
    }, () -> {
      shooting.setShooterPower(0);
      shooting.closeShooterInput();
    }, shooting);

    chassis.setDefaultCommand(new Drive(chassis, mainController));
    //shooting.setDefaultCommand(new SetTurnerDown(shooting));
    pickup.setDefaultCommand(new SetArm(pickup, Destination.UP).andThen(new StartEndCommand(() -> {}, () -> {}, pickup)));
    
    // Configure the button bindings

    autonomousChooser = new SendableChooser<>();
    autonomousChooser.setDefaultOption("Normal", Autonomouses.Normal);
    autonomousChooser.addOption("Wall", Autonomouses.Wall);
    autonomousChooser.addOption("Turn", Autonomouses.Turn);
    autonomousChooser.addOption("Quick", Autonomouses.Quick);

    SmartDashboard.putData("Autonomous", autonomousChooser);

    configureButtonBindings();
  }

  public enum Autonomouses {
    Wall,
    Normal,
    Turn,
    Quick
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
   * X button secondary -> open shackle
   * B button secondary -> default shoot
   * start button secondary -> start climb sequence
   * right joystick y secondary -> move elevator
   * Y button secondary -> close shackle
   * Back button main -> reverse controls
   * A button secondary -> reset position
   */
  private void configureButtonBindings() {
    aButtonMain.whenHeld(intake);
      
    yButtonMain.whileHeld(shoot);

    aButtonSecondary.whenPressed(new InstantCommand(() -> {
      NetworkTableInstance nt = NetworkTableInstance.getDefault();
      boolean isRed = nt.getEntry("FMSInfo/IsRedAlliance").getBoolean(true);
      SmartDashboard.putBoolean("isRed", isRed);
      Translation2d location = isRed ? ShootingUtil.RED_LAUNCH_LOCATION : ShootingUtil.BLUE_LAUNCH_LOCATION;
      chassis.setPose(new Pose2d(location.getX(), location.getY(), Rotation2d.fromDegrees(isRed ? 180 : 0)));
    }));
    
    xButtonMain.whenHeld(shoot2); //autoShoot

    xButtonSecondary.whileHeld(openShackle);

    yButtonSecondary.whileHeld(closeShackle);

    backButtonMain.whileHeld(new StartEndCommand(() -> {shooting.openShooterInput();}, () -> {shooting.closeShooterInput();}, shooting));

    startButtonSecondary.whenPressed(new InstantCommand(() -> {
      new SetArm(pickup, SetArm.Destination.DOWN).andThen(new StartEndCommand(() -> {}, () -> {}, pickup)).alongWith(new SetTurnerDown(shooting)).schedule();
      elivatorInside.changeClimbingMode();
      new Rainbow(ledHandler).schedule();
    }).andThen(new InstantCommand(() -> {chassis.setNeutralMode(elivatorInside.isClimbingMode());})));

    bButtonSecondary.whileHeld(throwOut);

    backButtonSecondary.whileHeld(new StartEndCommand(shooting::freeInput, shooting::closeShooterInput, shooting));
  }

  /*public Command getSimpleAutoCommand() {
    return new SetArm(pickup, Destination.DOWN).
      andThen(pickup.getIntakeCommand().
      raceWith(new MoveForward(chassis, 2))).
      andThen(new Shoot(shooting, chassis, Constants.SHOOTING_AUTO_VELOCITY, Constants.SHOOTING_AUTO_ANGLE));
  }*/

  /*public Command getAuto1Command() {
    Command start = new InstantCommand(chassis::setPosition1).andThen(
        new InstantCommand(() -> {chassis.setNeutralMode(true);}),
        new MoveForward(chassis, 0.7),
        (new Shoot(shooting, chassis).withTimeout(3)),
        new SetArm(pickup, Destination.DOWN));
    return start.andThen(
        (new MoveForward(chassis, 0.3).andThen(new Shoot(shooting, chassis)))
          .alongWith(pickup.getIntakeCommand()));
//      (pickup.getIntakeCommand().raceWith(new MoveForward(chassis, 0.3))),
//      (new Shoot(shooting, chassis).withTimeout(3)));
  }*/

  public Command getQuickShootCommand() {
    return new AngleForLow(shooting).andThen(new LowShoot(shooting, ledHandler).withTimeout(3),
      new MoveForward(chassis, 1)
    );
 }

  public Command getAutoLowShootCommand() {
    return new AngleForLow(shooting).alongWith(new SetArm(pickup, Destination.DOWN)).andThen(
       pickup.getIntakeCommand().raceWith(
          new LowShoot(shooting, ledHandler).withTimeout(2).andThen(
          new MoveForward(chassis, 0.8),
          new WaitCommand(1),
          new MoveForward(chassis, -0.8),
          new WaitCommand(1),
          new LowShoot(shooting, ledHandler).withTimeout(3)
        )), new MoveForward(chassis, 1.3).alongWith(new SetArm(pickup, Destination.UP)));
  }

  public Command getAutoLowShootCommand2() {
    return new AngleForLow(shooting).alongWith(new SetArm(pickup, Destination.DOWN)).andThen(
       pickup.getIntakeCommand().raceWith(
          new LowShoot(shooting, ledHandler).withTimeout(2).andThen(
          new MoveForward(chassis, 1),
          new WaitCommand(1),
          new MoveForward(chassis, -1),
          new WaitCommand(1),
          new LowShoot(shooting, ledHandler).withTimeout(3)
        )), new MoveForward(chassis, 1.3).alongWith(new SetArm(pickup, Destination.UP)));
  }

  public Command getAutoSpecial(){
    return new InstantCommand(() -> {chassis.setNeutralMode(true);}).andThen(
      new AngleForLow(shooting).alongWith(new SetArm(pickup, Destination.DOWN)), 
    pickup.getIntakeCommand().raceWith(new LowShoot(shooting, ledHandler).withTimeout(1.5).andThen(
      new Turn(chassis, -21), new MoveForward(chassis, 1.3), 
      new WaitCommand(1), new MoveForward(chassis, -1.3),
      new Turn(chassis, 21),
      new LowShoot(shooting, ledHandler).withTimeout(1.5)
    )), new MoveForward(chassis, 1.3).alongWith(new SetArm(pickup, Destination.UP)));
  }

  /*public Command getAuto2Command() {
    return (new MoveForward(chassis, 0.6).alongWith(new SetArm(pickup, Destination.DOWN))).andThen(
      (new Shoot(shooting, chassis).withTimeout(3)),
      (pickup.getIntakeCommand().raceWith(new MoveForward(chassis, 0.6))),
      (new Shoot(shooting, chassis).withTimeout(3)));
  }*/

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //return chassis.getAutoCommand("test1.wpilib.json");
    switch (autonomousChooser.getSelected()) {
      case Wall:
        System.out.println("Wall");
        return getAutoLowShootCommand();
      case Normal:
        System.out.println("Normal");
        return new InstantCommand(chassis::setPosition1).alongWith(getAutoLowShootCommand2());
      case Turn:
        System.out.println("Turn");
        return new InstantCommand(chassis::setPosition2).alongWith(getAutoSpecial());
      case Quick:
        return getQuickShootCommand();
      default:
        return null;
    }
  }

  public void onDisable() {
    chassis.setNeutralMode(false);
    new StartEndCommandOnDisable(() -> {}, () -> {}, ledHandler).schedule();
    ledHandler.setDefaultColor();
  }

  public void onTeleop() {
    ledHandler.setDefaultCommand(NetworkTableInstance.getDefault().getEntry("FMSInfo/IsRedAlliance").getBoolean(true) ?
        new MoveBetweenColors(142, 180, ledHandler) :
        new MoveBetweenColors(120, 142, ledHandler));
    chassis.setNeutralMode(false);
    ledHandler.getDefaultCommand().schedule();
    if (elivatorInside.isClimbingMode()) elivatorInside.changeClimbingMode();
  }

  public void onAuto() {
    ledHandler.setDefaultCommand(NetworkTableInstance.getDefault().getEntry("FMSInfo/IsRedAlliance").getBoolean(true) ?
        new MoveBetweenColors(142, 180, ledHandler) :
        new MoveBetweenColors(120, 142, ledHandler));
    ledHandler.getDefaultCommand().schedule();
    chassis.setNeutralMode(true);
  }
}
