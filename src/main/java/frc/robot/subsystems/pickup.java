// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class pickup extends SubsystemBase {

  WPI_TalonSRX midLeft =  new WPI_TalonSRX(Constants.midLeft);
  WPI_TalonSRX midRight = new WPI_TalonSRX(Constants.midRight);
  private double power;

  /** Creates a new BallIntake. */
  public pickup() {



    
  }

  @Override
  public void periodic() {

    public void setPower(double power){
       setPower(power);

    }
    // This method will be called once per scheduler run
  
    
  }
}
