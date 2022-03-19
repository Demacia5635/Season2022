// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.utils.FeedForward;
import frc.robot.subsystems.Chassis;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/** An example command that uses an example subsystem. */
public class Calibrate extends CommandBase {


  public static final double Power1 = 0.35;
  public static final double Power2 = 0.25;
  public static final double MinPower = 0.1;

  Chassis chassis;
  CalibrateTurn[] cmds;
  Command cmd;
  

  public Calibrate(Chassis chassis) {
    super();
    this.chassis = chassis;
    // create the list of Calibrate Turn Commands
    cmds = new CalibrateTurn[] {
      new CalibrateTurn(chassis, chassis::setPower, chassis::getLeftVelocity, chassis::getRightVelocity, Power1, Power1),
      new CalibrateTurn(chassis, chassis::setPower, chassis::getLeftVelocity, chassis::getRightVelocity, -Power1, -Power1),
      new CalibrateTurn(chassis, chassis::setPower, chassis::getLeftVelocity, chassis::getRightVelocity, Power1, MinPower),
      new CalibrateTurn(chassis, chassis::setPower, chassis::getLeftVelocity, chassis::getRightVelocity, -Power1, -MinPower),
      new CalibrateTurn(chassis, chassis::setPower, chassis::getLeftVelocity, chassis::getRightVelocity, Power2, Power2),
      new CalibrateTurn(chassis, chassis::setPower, chassis::getLeftVelocity, chassis::getRightVelocity, -Power2, -Power2),
    };
    // build the cmd to run all tests - with a 1 second wait in between
    cmd = null;
    for(Command c : cmds) {
      if(cmd == null) {
        cmd  = c;
      } else {
        // cmd = cmd.andThen().andThen(c);
        cmd = cmd.andThen(new WaitCommand(0.7)).andThen(c);
        System.out.println("RUNNING OVER THE COMMANDS WHICH ARENT");
      }
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // run the commnand
    cmd.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // nothing - waiting for the cmd o finish
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      // analyze the data
      // get the velocities - v1 = Power1/Power1 v2 = Power2/Power2, v1h = high of Power1/MinPower1 etc.
    double v1left = (cmds[0].lVelocity - cmds[1].lVelocity)/2;
    double v2left = (cmds[4].lVelocity - cmds[5].lVelocity)/2;
    double v1right = (cmds[0].rVelocity - cmds[1].rVelocity)/2;
    double v2right = (cmds[4].rVelocity - cmds[5].rVelocity)/2;
    double v1h = (cmds[2].lVelocity - cmds[3].lVelocity)/2;
    double v1l = (cmds[2].rVelocity - cmds[3].rVelocity)/2;
 
    // calculate the constants
    double kvl = (Power1 - Power2) / (v1left - v2left);
    double ksl = Power1 - v1left*kvl;
    double kvr = (Power1 - Power2) / (v1right - v2right);
    double ksr = Power1 - v1right*kvr;
    double ks = (ksl + ksr)/2;
    double kv = (kvl + kvr)/2;
    double hr = (Power1 - ks - kv * v1h)/(v1h - v1l); 
    double lr = (MinPower-ks-kv*v1l)/(v1h - v1l);
    // set up the global parameters - for the test
    FeedForward ff = new FeedForward();
    ff.K_S_L = ksl;
    ff.K_V_L = kvl;
    ff.K_S_R = ksr;
    ff.K_V_R = kvr;
    ff.K_H = hr;
    ff.K_L = lr;
    // check all values
    double error = 0;
    for(CalibrateTurn c : cmds) {
      ff.calculate(c.lVelocity, c.rVelocity);
      error = Math.abs(ff.leftP - c.lPower) + Math.abs(ff.rightP - c.rPower);
    }
    // put all data into the network table
    SmartDashboard.putNumber("Calibrate KS-L", ksl);
    SmartDashboard.putNumber("Calibrate KV-L", kvl);
    SmartDashboard.putNumber("Calibrate KS-R", ksr);
    SmartDashboard.putNumber("Calibrate KV-R", kvr);
    SmartDashboard.putNumber("Calibrate KS", ks);
    SmartDashboard.putNumber("Calibrate KV", kv);
    SmartDashboard.putNumber("Calibrate KH", hr);
    SmartDashboard.putNumber("Calibrate KL", lr);
    SmartDashboard.putNumber("Calibrate Error", error/12); // average of the 16 tests
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return cmd.isFinished();
  }
}