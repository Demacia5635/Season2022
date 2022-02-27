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
import frc.robot.commands.InstantCommandInDisable;
import frc.robot.utils.GroupOfMotors;

public class Chassis extends SubsystemBase{
  /** Creates a new Chassis. */
  private final GroupOfMotors left;
  private final GroupOfMotors right;
  private final PigeonIMU gyro;
  private static final SimpleMotorFeedforward aff = new SimpleMotorFeedforward(Constants.KS, Constants.KV);
  private boolean isBrake;
  private final DifferentialDriveOdometry odometry;

  public Chassis(){
    left = new GroupOfMotors(Constants.LEFT_FRONT_PORT, Constants.LEFT_BACK_PORT);
    right = new GroupOfMotors(Constants.RIGHT_FRONT_PORT, Constants.RIGHT_BACK_PORT);
    gyro = new PigeonIMU(Constants.GYRO_PORT);
    odometry = new DifferentialDriveOdometry(new Rotation2d(0));
    gyro.setFusedHeading(0);

    setNeutralMode(true);
    left.invertMotors(true);
    right.invertMotors(false);

    left.setK_P(Constants.KP);
    right.setK_P(Constants.KP);
  }

  public void setNeutralMode(boolean brake){
    isBrake = brake;
    left.setNeutralMode(isBrake);
    right.setNeutralMode(isBrake);
  }

  public void setAngularVelocity(double velocity, double turns){
    ChassisSpeeds speeds = new ChassisSpeeds(velocity * Constants.MAX_VELOCITY, 0, turns * Constants.MAX_ANGULAR_VELOCITY);

    DifferentialDriveWheelSpeeds wheelSpeeds = Constants.KINEMATICS.toWheelSpeeds(speeds);
    left.setVelocity(wheelSpeeds.leftMetersPerSecond, aff);
    right.setVelocity(wheelSpeeds.rightMetersPerSecond, aff);
  }
  /**
   * returns gyros position
   * @return
   */
  public double getFusedHeading(){
    return gyro.getFusedHeading();
  } 

  public double getAngle(){
    double heading = getFusedHeading() % 360;
    if (heading > 180) {
      heading -= 360;
    }
    else if (heading < -180) {
      heading += 360;
    }
    return heading;
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
    odometry.resetPosition(pose, Rotation2d.fromDegrees(getAngle()));
  }

  /**
   * Gets the average distance of the two encoders.
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (left.getDistance() + right.getDistance()) / 2;
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

  public Command getAutoCommand(String trajectoryFileName){
    Trajectory trajectory = new Trajectory();

    trajectoryFileName = "paths/" + trajectoryFileName;
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryFileName);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryFileName, ex.getStackTrace());
      return null;
    }

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

  public Command getAutCommand(Pose2d... poses){
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

  @Override
  public void periodic() {
    odometry.update(Rotation2d.fromDegrees(getFusedHeading()), left.getDistance(), right.getDistance());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
      SmartDashboard.putData("NeutralMode", new InstantCommandInDisable(() -> {setNeutralMode(!isBrake);}));
  }
}
