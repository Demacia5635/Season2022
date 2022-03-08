// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.Calibrate;
import frc.robot.commands.InstantCommandInDisable;
import frc.robot.utils.FeedForward;
import frc.robot.utils.GroupOfMotors;

public class Chassis extends SubsystemBase{
  /** Creates a new Chassis. */
  private final GroupOfMotors left;
  private final GroupOfMotors right;
  private final PigeonIMU gyro;
  private static final SimpleMotorFeedforward aff = new SimpleMotorFeedforward(Constants.KS / 12, Constants.KV / 12, Constants.KA / 12);
  private boolean isBrake;
  private final Field2d field2d;
  private final DifferentialDriveOdometry odometry;
  private final FeedForward feedForward = new FeedForward();
  private boolean isSagi = true;
  private boolean isReversed = false;
  private boolean isPicking = false;

  public Chassis(){
    left = new GroupOfMotors(Constants.LEFT_FRONT_PORT, Constants.LEFT_BACK_PORT);
    right = new GroupOfMotors(Constants.RIGHT_BACK_PORT, Constants.RIGHT_FRONT_PORT);
    gyro = new PigeonIMU(Constants.GYRO_PORT);
    odometry = new DifferentialDriveOdometry(new Rotation2d(0));
    setPosition1();
    field2d = new Field2d();

    setNeutralMode(false);
    left.invertMotors(false);
    right.invertMotors(true);

    left.setK_P(Constants.KP);
    right.setK_P(Constants.KP);
  }

  public void setPosition1() {
    resetEncoders();
    gyro.setFusedHeading(0);
    odometry.resetPosition(new Pose2d(5.8, 4.1, Rotation2d.fromDegrees(180)), new Rotation2d(0));
  }

  public void setNeutralMode(boolean brake){
    isBrake = brake;
    left.setNeutralMode(isBrake);
    right.setNeutralMode(isBrake);
  }

  public void changeNeutralMode() {
    setNeutralMode(!isBrake);
  }

  public void setAngularVelocity(double velocity, double turns){
    double absVel = Math.abs(velocity);
    if (isSagi) {
      turns *= (absVel == 0 ? Constants.ZERO_TURN_SAGI : Math.max(absVel, Constants.LOW_TURN_SAGI)) + Constants.TURN_SCALE_SAGI;
    }
    else
      turns *= absVel + Constants.TURN_SCALE_GUY;

    if (isPicking) velocity *= Constants.SCALE_VELOCITY_ON_PICKUP;
    ChassisSpeeds speeds = new ChassisSpeeds(velocity * Constants.MAX_VELOCITY, 0, turns * Constants.MAX_ANGULAR_VELOCITY);

    DifferentialDriveWheelSpeeds wheelSpeeds = Constants.KINEMATICS.toWheelSpeeds(speeds);
    setVelocityOurFF(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
  }

  public void setVelocityOurFF(double left, double right){
    feedForward.calculate(left, right); 
    this.left.setVelocity(left, feedForward.leftP);
    this.right.setVelocity(right, feedForward.rightP);
  }

  public double getLeftVelocity(){
    return left.getVelocity();
  }

  public double getRightVelocity(){
    return right.getVelocity();
  }

  public void reverse(boolean toReverse){
    isReversed = toReverse;
    left.invertMotors(isReversed);
    right.invertMotors(isReversed);
  }

  public boolean isReversed() {
    return isReversed;
  }

  /**
   * returns gyros position
   * @return
   */
  private double _getFusedHeading(){
    return gyro.getFusedHeading();
  } 

  private Rotation2d getHeading() {
    return getPose().getRotation();
  }

  public double getAngle(){
    return getHeading().getDegrees();
  }

  /**
   * sets power to the left side and right side
   * @param leftP
   * @param rightP
   */
  public void setPower(double leftP, double rightP){
    left.setPower(leftP);
    right.setPower(rightP);
  }
  /**
   * sets 
   * @param leftVelocityR
   * @param rightVelocityR
   */
  public void setVelocity(double leftVelocityR, double rightVelocityR){
    left.setVelocity(leftVelocityR, aff);
    right.setVelocity(rightVelocityR, aff);
  }

  /**
   * Returns the currently-estimated pose of the robot.
   * @return The pose.
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void setPose(double x, double y){
    resetOdometry(new Pose2d(x, y, Rotation2d.fromDegrees(getAngle())));
  }

  public void setPose(Pose2d pose){
    odometry.resetPosition(pose, Rotation2d.fromDegrees(_getFusedHeading()));
  }

  /**
   * Returns the current wheel speeds of the robot.
   * @return The current wheel speeds in m/s.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(left.getVelocity(), right.getVelocity());
  }

  public void resetEncoders() {
    left.resetEncoder();
    right.resetEncoder();
  }

  /**
   * Resets the odometry to the specified pose.
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, Rotation2d.fromDegrees(_getFusedHeading()));
  }

  public void resetOdometry(double x, double y, double heading) {
    System.out.printf("set odo tp %f  %f %f\n",x,y,heading);
    resetOdometry(new Pose2d(x,y, Rotation2d.fromDegrees(heading)));
  }

  /**
   * Gets the average distance of the two encoders.
   * @return the average of the two encoder readings in meters
   */
  public double getAverageEncoderDistance() {
    return (left.getDistance() + right.getDistance()) / 2;
  }

  /**
   * returns the angle to the ball
   * @return angle to the ball in degrees
   */
  public double getVisionAngle(){
    return SmartDashboard.getNumber("ball angle", Double.NaN);
  }

  /**
   * returns the distance to the ball
   * @return distance to the ball in meters
   */
  public double getVisionDistance(){
    return SmartDashboard.getNumber("ball distance", Double.NaN);
  }

  /**
   * goes to the ball according to vision
   * @param velocity the velocity to go at
   */
  public void goToBall(double velocity){
    double angle = getVisionAngle();
    double distance = getVisionDistance();

    if (angle == 0){
      setVelocity(velocity, velocity);
      return;
    }

    double radius = distance / (2 * Math.sin(Math.toRadians(angle)));

    double k = Constants.TRACK_WIDTH / 2;

    setVelocity(velocity * (1 + (k / radius)), velocity * (1 - (k / radius)));
  }

  public void setHeading(double angle) {
    gyro.setFusedHeading(angle);
  }

  /** Zeroes the heading of the robot. */
  public void resetGyro() {
    setHeading(0);
  }

  public void setVoltage(double lVoltage, double rVoltage) {
    left.setPower(lVoltage / 12);
    right.setPower(rVoltage / 12);
  }

  public static <T> List<T> arrayToList(T[] array){
    List<T> list = new ArrayList<>();
    for(T element : array){
      list.add(element);
    }
    return list;
  }

  public Trajectory getTrajectory(String trajectoryFileName){
    Trajectory trajectory = new Trajectory();

    trajectoryFileName = "output/" + trajectoryFileName;
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryFileName);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryFileName, ex.getStackTrace());
      return null;
    }
    
    return trajectory;
  }

  public Command getAutoCommand(String trajectoryFileName){
    Trajectory trajectory = new Trajectory();

    trajectoryFileName = "output/" + trajectoryFileName;
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryFileName);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryFileName, ex.getStackTrace());
      return null;
    }

    return getAutoCommand(trajectory);
  }

  public Command getAutoCommand(Trajectory trajectory){
    Command ramseteCommand =
        new RamseteCommand(
            trajectory,
            this::getPose,
            new RamseteController(Constants.RAMSETE_B, Constants.RAMSETE_ZETA),
            new SimpleMotorFeedforward(
                Constants.KS,
                Constants.KV,
                Constants.KA),
            Constants.KINEMATICS,
            this::getWheelSpeeds,
            new PIDController(Constants.KP * 12, 0, 0),
            new PIDController(Constants.KP * 12, 0, 0),
            // RamseteCommand passes volts to the callback
            this::setVoltage,
            this
    ).andThen(() -> {setPower(0, 0);}, this);

    return ramseteCommand;
  }

  public Command getAutoCommand(Pose2d... poses){
    DifferentialDriveVoltageConstraint autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                Constants.KS,
                Constants.KV,
                Constants.KA),
                Constants.KINEMATICS,
            10);

    TrajectoryConfig config =
        new TrajectoryConfig(
                Constants.MAX_VELOCITY_AUTO,
                Constants.MAX_ACCELERATION_AUTO)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Constants.KINEMATICS)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(arrayToList(poses), config);

    return getAutoCommand(trajectory);
  }

  public void setRobotPosition() {
    resetOdometry(SmartDashboard.getNumber("Set X Position", 4),
      SmartDashboard.getNumber("Set y Position", 4),
      SmartDashboard.getNumber("Set Heading", 0));
  }

  @Override
  public void periodic() {
    odometry.update(Rotation2d.fromDegrees(_getFusedHeading()), left.getDistance(), right.getDistance());
    field2d.setRobotPose(getPose());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    SmartDashboard.putData("NeutralMode", new InstantCommandInDisable(() -> {setNeutralMode(!isBrake);}));
    builder.addDoubleProperty("left encoder", left::getEncoder, null);
    builder.addDoubleProperty("right encoder", right::getEncoder, null);
    builder.addDoubleProperty("angle", this::getAngle, null);
    SmartDashboard.putData("Robot Position", field2d);
    double x = SmartDashboard.getNumber("Set X Position", 4);
    SmartDashboard.putNumber("Set X Position", x);
    x = SmartDashboard.getNumber("Set y Position", 4);
    SmartDashboard.putNumber("Set y Position", x);
    x = SmartDashboard.getNumber("Set Heading", 0);
    SmartDashboard.putNumber("Set Heading", x);
    SmartDashboard.putData("Set Position", new InstantCommandInDisable(
      ()->{setRobotPosition();}
    ));
    SmartDashboard.putData("Calibrate", new Calibrate(this));

    SmartDashboard.putData("Change Driver", new InstantCommandInDisable(() -> {isSagi = !isSagi;}));
    builder.addBooleanProperty("Is Sage", () -> {return isSagi;}, null);
  }
}
