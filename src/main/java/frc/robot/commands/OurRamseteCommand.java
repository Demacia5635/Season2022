// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Chassis;

/** Add your docs here. */
public class OurRamseteCommand extends CommandBase{
    private final Chassis chassis;
    private final Command command;
    private final Pose2d initialPose;

    public OurRamseteCommand(Chassis chassis, String... paths) {
        this.chassis = chassis;
        initialPose = chassis.getTrajectory(paths[0]).getInitialPose();
        command = SequentialCommandGroup.sequence(Arrays.stream(paths).map(chassis::getAutoCommand).toArray(Command[]::new));
    }

    @Override
    public void initialize() {
        chassis.setPose(initialPose);
        command.schedule();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted)
            command.cancel();
    }

    @Override
    public boolean isFinished() {
        return command.isFinished();
    }
}
