package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooting;

public class SetShootingAngle extends CommandBase {
    Shooting shooting;
    double angle = 30;
    double powerUp = -0.3;
    double powerDown = 0.4;

    boolean goingToSwitch = false;

    public SetShootingAngle(Shooting shooting) {
        this.shooting = shooting;   
        SmartDashboard.putNumber("Target Angle", angle);
    }
    
    @Override
    public void initialize() {
        angle = SmartDashboard.getNumber("Target Angle", 30);
        goingToSwitch = true;
        shooting.setTurnerPower(powerUp);
        SmartDashboard.putNumber("turner power", powerUp);
    }

    @Override
    public void execute() {
        SmartDashboard.putBoolean("To Switch",goingToSwitch);
        if(goingToSwitch) {
            if(shooting.getLimitSwitch()) {
                shooting.setTurnerPower(0);
                shooting.setTurnerAngle();
                goingToSwitch = false;
                SmartDashboard.putNumber("turner power", 0);
            }
        }
        if(!goingToSwitch) {
            double cangle = shooting.getTurnerAngle();
            if(angle < cangle) {
                shooting.setTurnerPower(powerDown);
                SmartDashboard.putNumber("turner power", powerUp);
            } else if(angle > cangle) {
                shooting.setTurnerPower(powerUp);
                SmartDashboard.putNumber("turner power", powerDown);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooting.setTurnerPower(0);
    }

    @Override
    public boolean isFinished() {
        if(goingToSwitch) {
            return false;
        }
        double cangle = shooting.getTurnerAngle();
        SmartDashboard.putNumber("Currect Angle", cangle);
        return Math.abs(cangle - angle) < 4;
    }
}
