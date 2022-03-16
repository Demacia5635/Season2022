package frc.robot.utils;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShootingUtil {

    public static final Translation2d TargetLocation = new Translation2d(8.3,4.1);
    public static final Translation2d RED_LAUNCH_LOCATION = new Translation2d(4, 5.5);
    public static final Translation2d BLUE_LAUNCH_LOCATION = new Translation2d(12.6, 2.8);
    public static final double yToAngle = 11.6 / 130;

    public static final LookUpTable xToDistance = new LookUpTable(new double[][]{
        { 364, 5},
        { 384, 4.5},
        { 410, 4},
        { 439, 3.5},
        { 478, 3},
        { 532, 2.5},
        { 608, 2},
    });

    public static final LookUpTable distanceToVelocityAndAngle = new LookUpTable(new double[][]{
        { 2.25, 13, 46},
        { 2.63, 14, 42},
        { 2.98, 14.5, 39},
        { 3.23, 16, 37},
        { 3.72, 17, 35},
        { 4.43, 18.7, 32},
        { 5, 19, 30.5}
    });
    
    public static double VisionXtoDistance(double x) {
        double distance = xToDistance.get(x)[0] + SmartDashboard.getNumber("Distance Change", 0);
        SmartDashboard.putNumber("Vision Distance", distance);
        return distance;
    }

    public static double[] distanceToVelocityAndAngle(double distance) {
        double[] shoot = distanceToVelocityAndAngle.get(distance); 
        shoot[1] += SmartDashboard.getNumber("Angle Change", 0);
        return shoot;
    }

    public static double distanceToVelocity(double distance) {
        double velocity = distanceToVelocityAndAngle.get(distance)[0];
        SmartDashboard.putNumber("Shooting Wanted Velocity", velocity);
        return velocity;
    }

    public static double distanceToAngle(double distance) {
        double angle = distanceToVelocityAndAngle.get(distance)[1];
        SmartDashboard.putNumber("Shooting Wanted Angle", angle);
        return angle;
    }

    public static double yToAngle(double y){
        double ans = (y - 270) * yToAngle + 4;
        SmartDashboard.putNumber("Vision Angle", ans);
        return -ans;
    }

    public static Translation2d getTargetPosition(Pose2d pose) {
        Translation2d t = TargetLocation.minus(pose.getTranslation());
        return t;
    }

    public static double getDistance(Translation2d t) {
        double distance = t.getNorm() - 0.61;
        SmartDashboard.putBoolean("Is In Range", distance <= 1.25 && distance >= 0.4);
        return distance;
    }

    public static Rotation2d getRotation(Translation2d t, Pose2d pose) {
        Rotation2d r = new Rotation2d(t.getX(), t.getY());
        double rot = r.getDegrees() - pose.getRotation().getDegrees() - 180;
        if(rot < -180) {
            rot += 360;
        } else if(rot > 180) {
            rot -= 360;
        }
        SmartDashboard.putBoolean("Angle In Range", Math.abs(rot) <= 10);
        return Rotation2d.fromDegrees(rot);
    }
}
