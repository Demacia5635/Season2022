// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.FeedForward;
import frc.robot.vision.Processor;

public class Shooting extends SubsystemBase {
  /** Creates a new Shooting. */

  private final WPI_TalonFX shooter;
  private final FeedForward shooterAff;
  private final Processor processor;

  public Shooting() {
    shooter = new WPI_TalonFX(Constants.SHOOTER_PORT);
    shooterAff = new FeedForward(Constants.SHOOTER_KS, Constants.SHOOTER_KV);
    processor = new Processor(Constants.CAMERA_WIDTH_PIXELS, Constants.CAMERA_HEIGHT_PIXELS);
  }

  /**
   * sets the power of the shooter wheel
   * @param power between 1 and -1
   */
  public void setShooterPower(double power){
    shooter.set(ControlMode.PercentOutput, power);
  }

  /**
   * sets the velocity of the shooter wheel by feedforward
   * @param velocity in meter/sec
   */
  public void setShooterVelocity(double velocity){
    shooter.set(ControlMode.Velocity, velocity * 10 / Constants.SHOOTER_PULSE_TO_METER, 
        DemandType.ArbitraryFeedForward, shooterAff.get(velocity));
  }

  /**
   * gets the shooter velocity
   * @return in meter/sec
   */
  public double getShooterVelocity(){
    return shooter.getSelectedSensorPosition() * Constants.SHOOTER_PULSE_TO_METER / 10;
  }

  /**
   * stop feeding cargo to the shooter mechanism
   */
  public void closeShooterInput(){
    //TODO : add implementation
  }

  /**
   * feed cargo to the shooter mechanism
   */
  public void openShooterInput(){
    //TODO : add implementation
  }

  /**
   * returns the distance and the angle from the vision process
   * @return a Vector2d where x=distance, y=angle
   */
  public Vector2d getVision(){
    processor.run();
    return new Vector2d(processor.get("distance"), processor.get("angle"));
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Shooter Velocity", this::getShooterVelocity, this::setShooterVelocity);
    processor.initSendable(builder);
  }
}
