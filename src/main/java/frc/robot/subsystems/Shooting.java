// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.AngleForLow;
import frc.robot.utils.LookUpTable;

public class Shooting extends SubsystemBase {
  /** Creates a new Shooting. */

  private final WPI_TalonFX shooterMain;
  private final WPI_TalonFX shooterSecondary;
  private final SimpleMotorFeedforward shooterAff;
  private final WPI_TalonSRX inputWheel;
  private final WPI_TalonSRX turner;
  private final DigitalInput upperLimitSwitch;
  private final DigitalInput lowerLimitSwitch;
  private static final LookUpTable distanceToVelocity = new LookUpTable(new double[][] {
    {0.3, 7.5},
    {0.9, 8.1},
    {1.1, 8.2},
    {1.3, 8.6},
    {1.5, 8.8},
    {1.65, 9.1}
  });

  public Shooting() {
    shooterMain = new WPI_TalonFX(Constants.SHOOTER_PORT_MAIN);
    turner = new WPI_TalonSRX(Constants.TURNER_PORT);
    turner.setNeutralMode(NeutralMode.Brake);
    shooterAff = new SimpleMotorFeedforward(Constants.SHOOTER_KS, Constants.SHOOTER_KV);
    shooterSecondary = new WPI_TalonFX(Constants.SHOOTER_PORT_SECONDARY);
    inputWheel = new WPI_TalonSRX(Constants.INPUT_WHEEL_PORT);
    shooterSecondary.setInverted(true);
    upperLimitSwitch = new DigitalInput(Constants.UPPER_LIMIT_SWITCH_PORT);
    lowerLimitSwitch = new DigitalInput(Constants.LOWER_LIMIT_SWITCH_PORT);
    shooterMain.config_kP(0, Constants.SHOOTER_KP);
    shooterSecondary.config_kP(0, Constants.SHOOTER_KP);
    inputWheel.setInverted(false);
    inputWheel.setSensorPhase(true);
    inputWheel.setSelectedSensorPosition(0);
  }

  /**
   * sets the power of the shooter wheel
   * @param power between 1 and -1
   */
  public void setShooterPower(double power){
    shooterMain.set(ControlMode.PercentOutput, power);
    shooterSecondary.set(ControlMode.PercentOutput, power);
  }

  /**
   * sets the power of the turner
   * @param power between 1 and -1
   */
  public void setTurnerPower(double power){
    if ((getUpperLimitSwitch() && power < 0) || (getLowerLimitSwitch() && power > 0)){
      turner.set(ControlMode.PercentOutput, 0);
    }
    else turner.set(ControlMode.PercentOutput, power);
  }

  /**
   * sets the velocity of the shooter wheel by feedforward
   * @param velocity in meter/sec
   */
  public void setShooterVelocity(double velocity){
    shooterSecondary.set(ControlMode.Velocity, velocity / (10 * Constants.SHOOTER_PULSE_TO_METER), 
        DemandType.ArbitraryFeedForward, shooterAff.calculate(velocity));
    velocity *= 1 + Constants.SPIN_PERCENTAGE;
    shooterMain.set(ControlMode.Velocity, velocity / (10 * Constants.SHOOTER_PULSE_TO_METER), 
        DemandType.ArbitraryFeedForward, shooterAff.calculate(velocity));
    
  }

  /**
   * gets the shooter velocity
   * @return in meter/sec
   */
  public double getShooterVelocity(){
    return shooterMain.getSelectedSensorVelocity() * Constants.SHOOTER_PULSE_TO_METER * 10;
  }

  public double getShooterVelocity2(){
    return shooterSecondary.getSelectedSensorVelocity() * Constants.SHOOTER_PULSE_TO_METER * 10;
  }

  public double getShooterEncoder(){
    return shooterMain.getSelectedSensorPosition();
  }

  /**
   * returns the limit switch state
   * @return true if the limit switch is closed
   */
  public boolean getUpperLimitSwitch() {
    return !upperLimitSwitch.get();
  }

  public boolean getLowerLimitSwitch() {
    return !lowerLimitSwitch.get();
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

  public void freeInput() {
    inputWheel.set(ControlMode.PercentOutput, -Constants.INPUT_WHEEL_POWER);
  }

  public double getTurnerPower(){
    return turner.get();
  }
  
  @Override
  public void periodic() {
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Shooter Velocity", this::getShooterVelocity, this::setShooterVelocity);
    builder.addBooleanProperty("upper limit switch", this::getUpperLimitSwitch, null);
    builder.addBooleanProperty("lower limit switch", this::getLowerLimitSwitch, null);
    // builder.addDoubleProperty("target direction", this::getTargetDirection, null);
    builder.addDoubleProperty("Shooting Velocity 2", this::getShooterVelocity2, null);

    SmartDashboard.putNumber("Distance Change", SmartDashboard.getNumber("Distance Change", 0));
    SmartDashboard.putNumber("Angle Change", SmartDashboard.getNumber("Angle Change", 0));
    SmartDashboard.putData("Set Low Angle", new AngleForLow(this));
  }

  public boolean visionOK() {
    return SmartDashboard.getBoolean("vision_found", false);
  }

  public double getShootingVelocity(double distance) {
    distance -= Constants.DISTANCE_FRON_HUB_TO_FENDER + Constants.ROBOT_LENGTH / 2;
    return distanceToVelocity.get(distance)[0];
  }
}
