// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.InstantCommandInDisable;
import frc.robot.commands.MoveElivator;

public class ElivatorInside extends SubsystemBase {
  /** Creates a new Elivator_Inside. */

  private final WPI_TalonFX telescopicMotor;
  private final WPI_TalonSRX shackleOpenner;
  private final MoveElivator command;
  private boolean isClimbingMode = false;

  public ElivatorInside(XboxController controller) {
    this.telescopicMotor = new WPI_TalonFX(Constants.TELESCOPIC_MOTOR);
    this.shackleOpenner = new WPI_TalonSRX(Constants.SHACKLE_OPENNER);

    telescopicMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 40, 0.1));
    command = new MoveElivator(this, controller);
    setDefaultCommand(command);

    //telescopicMotor.setSelectedSensorPosition(0);
  }

  /**
   * sets the power of telescopic motor 
   */
  public void setPowerTelescopicMotor(double power){ 
    this.telescopicMotor.set(ControlMode.PercentOutput, power);
  }

  /**
   * sets the power of shackleOpenner motor
   */
  public void setPowerShackleOpenner(double power){ 
    this.shackleOpenner.set(ControlMode.PercentOutput, power);
  }

/**
 * 
 * @return the current pulses from the encoder 
 */
  public double getSelectedSensorPosition(){
    return telescopicMotor.getSelectedSensorPosition();
  }
  /**
   * transfers the total pulses of the encoder to meter
   * @return
   */
  public double getSelectedSensorPositionInMeters(){
      return getSelectedSensorPosition() / Constants.PULSES_PER_METER;
  }

  public void changeClimbingMode(){
    isClimbingMode = !isClimbingMode;
  }

  public boolean isClimbingMode() {
    return isClimbingMode;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void initSendable(SendableBuilder builder) {
      SmartDashboard.putData("Reverse Elevator Motor", new InstantCommandInDisable(() -> {telescopicMotor.setInverted(!telescopicMotor.getInverted());}));
  }
}
