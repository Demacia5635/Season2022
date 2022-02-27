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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.InstantCommandInDisable;
import frc.robot.utils.GroupOfMotors;

public class Chassis extends SubsystemBase{
  /** Creates a new Chassis. */
  private final GroupOfMotors left;
  private final GroupOfMotors right;
  private final PigeonIMU gyro;
  private final SimpleMotorFeedforward aff;
  private boolean isBrake;
  private final DifferentialDriveOdometry odometry;

  public Chassis(){
    left = new GroupOfMotors(Constants.LEFT_FRONT_PORT, Constants.LEFT_BACK_PORT);
    right = new GroupOfMotors(Constants.RIGHT_FRONT_PORT, Constants.RIGHT_BACK_PORT);
    gyro = new PigeonIMU(Constants.GYRO_PORT);
    odometry = new DifferentialDriveOdometry(new Rotation2d(0));
    gyro.setFusedHeading(0);

    aff = new SimpleMotorFeedforward(Constants.KS, Constants.KV);

    setNeutralMode(true);
    left.invertMotors(true);
    right.invertMotors(false);

    left.setK_P(Constants.KP);
    right.setK_P(Constants.KP);
  }

  public void setPower(double lPower, double rPower){
    left.setPower(lPower);
    right.setPower(rPower);
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void initSendable(SendableBuilder builder) {
      SmartDashboard.putData("NeutralMode", new InstantCommandInDisable(() -> {setNeutralMode(!isBrake);}));
  }
}
