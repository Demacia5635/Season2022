// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.LedHandler;

public class LedTransition extends CommandBase {
  /** Creates a new LedTransition. */
  private final Supplier<double[]> hsvSupplier;
  private final DoubleSupplier durationSupplier;
  private double targetH, targetS, targetV, duration, startH, startS, startV;
  private int time = 0;

  public LedTransition(Supplier<double[]> hsvSupplier, DoubleSupplier durationSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.hsvSupplier = hsvSupplier;
    this.durationSupplier = durationSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double[] hsv = hsvSupplier.get();
    targetH = hsv[0];
    targetS = hsv[1];
    targetV = hsv[2];
    hsv = LedHandler.getHsv();
    startH = hsv[0];
    startS = hsv[1];
    startV = hsv[2];
    duration = durationSupplier.getAsDouble() * 1000;
    System.out.println("Transition:\n\tTargetH: " + targetH + "\n\tTargetS: " + targetS + "\n\tTargetV: " + targetV);
    time = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentHue = (startH + (targetH - startH) * time / duration);
    double currentSaturation = (startS + (targetS - startS) * time / duration);
    double currentValue = (startV + (targetV - startV) * time / duration);
    LedHandler.setHsv(currentHue, currentSaturation, currentValue);
    time += 20;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    LedHandler.setHsv(targetH, targetS, targetV);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return time > duration;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
