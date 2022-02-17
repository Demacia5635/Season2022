// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
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
  private final DigitalInput limitSwitch;

  public Shooting() {
    shooterMain = new WPI_TalonFX(Constants.SHOOTER_PORT_MAIN);
    turner = new WPI_TalonSRX(Constants.TURNER_PORT);
    turner.setNeutralMode(NeutralMode.Brake);
    shooterAff = new FeedForward(Constants.SHOOTER_KS, Constants.SHOOTER_KV);
    shooterSecondary = new WPI_TalonFX(Constants.SHOOTER_PORT_SECONDARY);
    inputWheel = new WPI_TalonSRX(Constants.INPUT_WHEEL_PORT);
    shooterSecondary.setInverted(true);
    shooterSecondary.follow(shooterMain);
    limitSwitch = new DigitalInput(Constants.LIMIT_SWITCH_PORT);
    shooterMain.config_kP(0, Constants.SHOOTER_KP);
    inputWheel.setSensorPhase(true);
    inputWheel.setSelectedSensorPosition(0);
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
    if (getLimitSwitch() && power < 0){
      turner.set(ControlMode.PercentOutput, 0);
    }
    else turner.set(ControlMode.PercentOutput, power);
  }

  /**
   * sets the velocity of the shooter wheel by feedforward
   * @param velocity in meter/sec
   */
  public void setShooterVelocity(double velocity){
    shooterMain.set(ControlMode.Velocity, velocity / (10 * Constants.SHOOTER_PULSE_TO_METER), 
        DemandType.ArbitraryFeedForward, shooterAff.get(velocity));
  }

  /**
   * gets the shooter velocity
   * @return in meter/sec
   */
  public double getShooterVelocity(){
    return shooterMain.getSelectedSensorVelocity() * Constants.SHOOTER_PULSE_TO_METER * 10;
  }

  public double getShooterEncoder(){
    return shooterMain.getSelectedSensorPosition();
  }

  /**
   * gets the turner angle
   * @return in degrees
   */
  public double getTurnerAngle() {
    return inputWheel.getSelectedSensorPosition() * Constants.PULSE_TO_ANGLE;
  }

  /**
   * returns the limit switch state
   * @return true if the limit switch is closed
   */
  public boolean getLimitSwitch(){
    return limitSwitch.get();
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

  public double getTurnerPower(){
    return turner.get();
  }
  
  @Override
  public void periodic() {
    if (getLimitSwitch() && getTurnerPower() < 0){
      setTurnerPower(0);
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Shooter Velocity", this::getShooterVelocity, this::setShooterVelocity);
    builder.addDoubleProperty("Angle", this::getTurnerAngle, null);
    builder.addDoubleProperty("encoder", this::getShooterEncoder, null);
    builder.addBooleanProperty("limit switch", this::getLimitSwitch, null);
  }
}
