// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.SetArm;

public class Pickup extends SubsystemBase {

  private final WPI_TalonSRX intake, arm;
  //private final ArmFeedforward armFeedforward;

  /** Creates a new BallIntake. */
  private boolean isDown;

  public Pickup() {
    intake = new WPI_TalonSRX(Constants.INTAKE_PORT);
    arm = new WPI_TalonSRX(Constants.ARM_PORT);
    arm.setInverted(false);
    isDown = false;

    //armFeedforward = new ArmFeedforward(Constants.GRIPPER_KS, Constants.GRIPPER_KCOS, Constants.GRIPPER_KV);
    
    //arm.setSelectedSensorPosition(Constants.TOP_ARM_ANGLE * Constants.ARM_PULSES_PER_ROTATION / 360);
  }

  /**
   * Sets the power of the intake.
   */
  public void setPower(double power){
    intake.set(ControlMode.PercentOutput, power);
  }

  /**
   * the limit switch is pressed when the arm is at the top
   * @return true if the limit switch is pressed
   */
  public boolean getUpperLimit(){
    return arm.isRevLimitSwitchClosed() != 0;
  }

  /**
   * the limit switch is pressed when the arm is at the bottom
   * @return true if the limit switch is pressed
   */
  public boolean getLowerLimit(){
    return arm.isFwdLimitSwitchClosed() != 0;
  }

  public boolean isDown(){
    return isDown;
  }
  
  /**
   * Sets the velocity of the arm
   * @param velocity The velocity to set the arm to in rad/s
   */
  /*public void setVelocity(double velocity){
    arm.set(ControlMode.PercentOutput, armFeedforward.calculate(Math.toRadians(getAngle()), velocity));
  }*/

  /**
   * Returns the current angle of the arm in degrees.
   */
  /*public double getAngle(){
    return arm.getSelectedSensorPosition() * ( 360 / Constants.ARM_PULSES_PER_ROTATION);
  }*/

  public void setArmPower(double power){
    arm.set(ControlMode.PercentOutput, power);
  }

  @Override
  public void periodic() {
    if (getLowerLimit()) isDown = true;
    if (getUpperLimit()) isDown = false;
    // This method will be called once per scheduler run
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addBooleanProperty("Lower Limit Switch", this::getLowerLimit, null);
    builder.addBooleanProperty("Upper Limit Switch", this::getUpperLimit, null);
    builder.addBooleanProperty("Is Down", this::isDown, null);
    builder.addDoubleProperty("Arm Current", arm::getStatorCurrent, null);
  }
  
  public CommandBase getIntakeCommand() {
    return new SetArm(this, SetArm.Destination.DOWN).raceWith(new StartEndCommand(
      () -> {setPower(Constants.PICKUP_POWER);},() -> {setPower(0);})).andThen(new StartEndCommand(
        () -> {setPower(Constants.PICKUP_POWER);},() -> {setPower(0);}, this));
  }
}
