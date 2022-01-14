// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.rmi.ConnectIOException;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.GroupOfMotors;

public class Chassis extends SubsystemBase{
  /** Creates a new Chassis. */
  private final GroupOfMotors left;
  private final GroupOfMotors right;
  private final PigeonIMU gyro; 

  public Chassis() {
    left = new GroupOfMotors(Constants.LEFT_FRONT_PORT, Constants.LEFT_BACK_PORT);
    right = new GroupOfMotors(Constants.RIGHT_FRONT_PORT, Constants.RIGHT_BACK_PORT);
    gyro = new PigeonIMU(Constants.GYRO_PORT);
  }
  
  /**
   * returns gyros position
   * @return
   */
  public double getAngle(){
    return gyro.getFusedHeading();
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
    SimpleMotorFeedforward Aff = new SimpleMotorFeedforward(Constants.KS,Constants.KV,Constants.KA);
    left.setRelVelocity(leftVelocityR,Aff);
    right.setRelVelocity(rightVelocityR,Aff);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
