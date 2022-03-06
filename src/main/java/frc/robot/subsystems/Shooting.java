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
import frc.robot.commands.ShootingCalibration;
import frc.robot.utils.ShootingUtil;

public class Shooting extends SubsystemBase {
  /** Creates a new Shooting. */

  private final WPI_TalonFX shooterMain;
  private final WPI_TalonFX shooterSecondary;
  private final SimpleMotorFeedforward shooterAff;
  private final WPI_TalonSRX inputWheel;
  private final WPI_TalonSRX turner;
  private final DigitalInput limitSwitch;
  private final ShootingCalibration calibration;
  private final Chassis chassis;

  public Shooting(Chassis chassis) {
    this.chassis = chassis;
    shooterMain = new WPI_TalonFX(Constants.SHOOTER_PORT_MAIN);
    turner = new WPI_TalonSRX(Constants.TURNER_PORT);
    turner.setNeutralMode(NeutralMode.Brake);
    shooterAff = new SimpleMotorFeedforward(Constants.SHOOTER_KS, Constants.SHOOTER_KV);
    shooterSecondary = new WPI_TalonFX(Constants.SHOOTER_PORT_SECONDARY);
    inputWheel = new WPI_TalonSRX(Constants.INPUT_WHEEL_PORT);
    shooterSecondary.setInverted(true);
    limitSwitch = new DigitalInput(Constants.LIMIT_SWITCH_PORT);
    shooterMain.config_kP(0, Constants.SHOOTER_KP);
    shooterSecondary.config_kP(0, Constants.SHOOTER_KP);
    inputWheel.setInverted(false);
    inputWheel.setSensorPhase(true);
    inputWheel.setSelectedSensorPosition(0);

    calibration = new ShootingCalibration(this, chassis);
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
   * gets the turner angle
   * @return in degrees
   */
  public double getTurnerAngle() {
    return inputWheel.getSelectedSensorPosition() * Constants.PULSE_TO_ANGLE;
  }

  public void setTurnerAngle() {
    inputWheel.setSelectedSensorPosition(52.0 / Constants.PULSE_TO_ANGLE);
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
   * gets the angle from the vision
   * @return the angle value from vision
   */
  public double getVisionAngle(){
    return ShootingUtil.yToAngle(SmartDashboard.getNumber("vision_tower_y", Double.NaN)) * 544 / 480;
  }

  public double getVisionDistance(){
    return Constants.CAMERA_TOWER_DIFF / Math.tan(Math.toRadians(getVisionAngle() + Constants.CAMERA_ANGLE));
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
    builder.addDoubleProperty("Angle", this::getTurnerAngle, null);
    builder.addDoubleProperty("encoder", this::getShooterEncoder, null);
    builder.addBooleanProperty("limit switch", this::getLimitSwitch, null);
    builder.addBooleanProperty("Viusion OK", this::visionOK, null);
    builder.addDoubleProperty("target direction", this::getTargetDirection, null);
    builder.addDoubleProperty("target distance", this::getTargetDistance, null);
    builder.addDoubleProperty("shoot velocity", this::getShootingVelocity, null);
    builder.addDoubleProperty("Shooting Velocity 2", this::getShooterVelocity2, null);
    builder.addDoubleProperty("shoot angle", this::getShootingAngle, null);
    

    SmartDashboard.putData("Start Calibration", calibration);
  }

  public boolean visionOK() {
    return SmartDashboard.getBoolean("vision_found", false);
  }

  public double getTargetDistance() {
    if(visionOK()) {
      return ShootingUtil.VisionXtoDistance(getVisionX());
    } else {
      return ShootingUtil.getDistance(ShootingUtil.getTargetPosition(chassis.getPose()));
    }
  }
  public double getTargetDirection() {
    if(visionOK()) {
      return getVisionAngle();
    } else {
      return ShootingUtil.getRotation(
        ShootingUtil.getTargetPosition(chassis.getPose())
        , chassis.getPose()).getDegrees();
    }
  }
  public double getShootingVelocity() {
    return ShootingUtil.distanceToVelocity(getTargetDistance());
  }
  public double getShootingAngle() {
    return ShootingUtil.distanceToAngle(getTargetDistance());
  }


}
