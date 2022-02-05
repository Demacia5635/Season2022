// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.FeedForward;

public class Shooting extends SubsystemBase {
  /** Creates a new Shooting. */

  private final WPI_TalonFX shooterMain;
  private final WPI_TalonFX shooterSecondary;
  private final FeedForward shooterAff;
  private final WPI_TalonSRX inputWheel;
  private final WPI_TalonSRX turner;
  private final PigeonIMU gyro;

  public Shooting() {
    shooterMain = new WPI_TalonFX(Constants.SHOOTER_PORT_MAIN);
    turner = new WPI_TalonSRX(Constants.TURNER_PORT);
    turner.setNeutralMode(NeutralMode.Brake);
    shooterAff = new FeedForward(Constants.SHOOTER_KS, Constants.SHOOTER_KV);
    gyro = new PigeonIMU(Constants.TURNER_GYRO_PORT);
    shooterSecondary = new WPI_TalonFX(Constants.SHOOTER_PORT_SECONDARY);
    inputWheel = new WPI_TalonSRX(Constants.INPUT_WHEEL_PORT);
    shooterSecondary.setInverted(true);
    shooterSecondary.follow(shooterMain);
  }

  /**
   * sets the power of the shooter wheel
   * @param power between 1 and -1
   */
  public void setShooterPower(double power){
    shooterMain.set(ControlMode.PercentOutput, power);
  }

  /**
   * sets the power of the turner
   * @param power between 1 and -1
   */
  public void setTurnerPower(double power){
    turner.set(ControlMode.PercentOutput, power);
  }

  /**
   * sets the velocity of the shooter wheel by feedforward
   * @param velocity in meter/sec
   */
  public void setShooterVelocity(double velocity){
    shooterMain.set(ControlMode.Velocity, velocity * 10 / Constants.SHOOTER_PULSE_TO_METER, 
        DemandType.ArbitraryFeedForward, shooterAff.get(velocity));
  }

  /**
   * gets the shooter velocity
   * @return in meter/sec
   */
  public double getShooterVelocity(){
    return shooterMain.getSelectedSensorPosition() * Constants.SHOOTER_PULSE_TO_METER / 10;
  }

  /**
   * gets the turner angle
   * @return in degrees
   */
  public double getTurnerAngle(){
    //TODO : check gyro
    return -1;
  }

  /**
   * stop feeding cargo to the shooter mechanism
   */
  public void closeShooterInput(){
    inputWheel.set(ControlMode.PercentOutput, 0);
  }

  /**
   * feed cargo to the shooter mechanism
   */
  public void openShooterInput(){
    inputWheel.set(ControlMode.PercentOutput, Constants.INPUT_WHEEL_POWER);
  }

  /**
   * gets the x from the vision
   * @return the x value from vision
   */
  public double getVisionX(){
    return SmartDashboard.getNumber("vision_tower_x", Double.NaN);
  }

  /**
   * gets the y from the vision
   * @return the y value from vision
   */
  public double getVisionY(){
    return SmartDashboard.getNumber("vision_tower_y", Double.NaN);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Shooter Velocity", this::getShooterVelocity, this::setShooterVelocity);
  }
}