// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Shooting;

public class Shoot extends CommandBase {
  
  private final Shooting shooting;
  private final DoubleSupplier velocityGetter;
  private final DoubleSupplier angleGetter;
  private final DoubleSupplier headingGetter;
  private boolean shoot, isAngleOK, velocityOK, headingOK;
  private final Chassis chassis;

  private final double turnerPowerUp = -0.3;
  private final double turnerPowerDown = 0.3;
  private boolean toSwitch = false;
  private int timeout;

  public Shoot(Shooting shooting, Chassis chassis, 
      DoubleSupplier velocityGetter, DoubleSupplier angleGetter, DoubleSupplier headingGetter) {
    this.shooting = shooting;
    this.chassis = chassis;
    this.velocityGetter = velocityGetter;
    this.angleGetter = angleGetter;
    this.headingGetter = headingGetter;
    shoot = true;
    isAngleOK = false;
    headingOK = false;
    velocityOK = false;

    addRequirements(shooting, chassis);
  }

  public Shoot(Shooting shooting, Chassis chassis) {
    this(shooting, chassis, 
       ()->{return shooting.getShootingVelocity();},
       ()->{return shooting.getShootingAngle();},
       ()->{return shooting.getTargetDirection();});
  }
  public Shoot(Shooting shooting, Chassis chassis, double velocity, double angle) {
    this(shooting, chassis, () -> {return velocity;}, () -> {return angle;}, () -> {return 0;});
    shoot = true;
  }

  @Override
  public void initialize() {
    isAngleOK = false;
    headingOK = false;
    velocityOK = false;
    toSwitch = true;
    shooting.setTurnerPower(turnerPowerUp);
  }

  public void shoot(){
    shoot = true;
  }

  private void setHeading() {
    double h = headingGetter.getAsDouble();
    double power = h * Constants.ANGLE_KP;
    SmartDashboard.putNumber("Turn Power", power);
    if(Math.abs(h) > Constants.MAX_ANGLE_ERROR_CHASSIS) {
      chassis.setPower(-power, power);
      headingOK = false;
    } else {
        chassis.setPower(0,0);
        headingOK = true;
    }
  }

  private void setVelocity() {
    double v = velocityGetter.getAsDouble();
    SmartDashboard.putNumber("Current Shooting Velocity", v);
    shooting.setShooterVelocity(v);
    double cv = shooting.getShooterVelocity2();
    velocityOK = Math.abs(v-cv) < Constants.MAX_SHOOT_VELOCITY_ERROR;
  }

  private void setAngle() {
    double angle = angleGetter.getAsDouble();
    SmartDashboard.putNumber("Current Shooting Angle", angle);
    // handle angle - to switch
    if(toSwitch) {
      if(shooting.getLimitSwitch()) { // switch reached, set angle and reverse power
        shooting.setTurnerAngle();
        timeout = 0;
        toSwitch = false;
        isAngleOK = false;
      }
    }
    // handle angle after switch
    if(!toSwitch && !isAngleOK) {
      if (timeout < 25) {
        timeout++;
      }
      else {
        double currentAngle = shooting.getTurnerAngle();
        if(currentAngle < angle + Constants.MAX_SHOOT_ANGLE_ERROR) {
          // angle OK
          isAngleOK= true;
          shooting.setTurnerPower(0);
        }
        else {
          shooting.setTurnerPower(turnerPowerDown);
        }
      }
    }
  }

  @Override
  public void execute() {
    setHeading();
    setVelocity();
    setAngle();
    SmartDashboard.putBoolean("Angle Correct", isAngleOK);

    if(isAngleOK && velocityOK && headingOK && shoot) {
        shooting.openShooterInput();
    }


/*    if (!angleRight && Math.abs(angleGetter.getAsDouble() - shooting.getTurnerAngle()) > ){
      shooting.setTurnerPower(Math.signum(angleGetter.getAsDouble() - shooting.getTurnerAngle()) * Constants.TURNER_DEFAULT_POWER);
    }
    else if (shoot && 
            Math.abs(shooting.getShooterVelocity() - velocityGetter.getAsDouble()) < 
            Constants.MAX_SHOOT_VELOCITY_ERROR){
      shooting.setTurnerPower(0);
      shooting.openShooterInput();
      angleRight = true;
    }
    else {
      angleRight = true;
      shooting.setTurnerPower(0);
    }
    */
  }

  @Override
  public void end(boolean interrupted) {
    shooting.closeShooterInput();
    shooting.setShooterPower(0);
    shooting.setTurnerPower(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
