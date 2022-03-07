package frc.robot.utils;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShootingUtil {

    public static final Translation2d TargetLocation = new Translation2d(8.3,4.1);
    public static final Translation2d RED_LAUNCH_LOCATION = new Translation2d(3.85, 5.5);
    public static final Translation2d BLUE_LAUNCH_LOCATION = new Translation2d(12.75, 2.8);
    public static final double yToAngle = 11.6 / 130;

    public static final LookUpTable xToDistance = new LookUpTable(new double[][]{
        { 356, 5},
        { 372, 4.5},
        { 395, 4},
        { 423, 3.5},
        { 455, 3},
        { 505, 2.5},
        { 571, 2},
        { 622, 1.65}
    });

    public static final LookUpTable distanceToVelocityAndAngle = new LookUpTable(new double[][]{
        { 2, 12, 52},
        { 2.5, 14, 50},
        { 3, 15.5, 46},
        { 3.5, 17, 43.5},
        { 4, 18.5, 37},
        { 4.5, 20, 35},
        { 5, 21, 33},
        { 5.5, 22, 31},
        { 6, 24, 30},
        { 7, 27, 25}
    });
    
    public static double VisionXtoDistance(double x) {
        double distance = xToDistance.get(x)[0];
        SmartDashboard.putNumber("Vision Distance", distance);
        return distance;
    }

    public static double[] distanceToVelocityAndAngle(double distance) {
        return distanceToVelocityAndAngle.get(distance);
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
        double ans = (y - 270) * yToAngle + 9;
        SmartDashboard.putNumber("Vision Angle", ans);
        return -ans;
    }

    public static Translation2d getTargetPosition(Pose2d pose) {
        Translation2d t = TargetLocation.minus(pose.getTranslation());
        return t;
    }

    public static double getDistance(Translation2d t) {
        return t.getNorm() - 0.61;
    }

    public static Rotation2d getRotation(Translation2d t, Pose2d pose) {
        Rotation2d r = new Rotation2d(t.getX(), t.getY());
        double rot = r.getDegrees() - pose.getRotation().getDegrees() - 180;
        if(rot < -180) {
            rot += 360;
        } else if(rot > 180) {
            rot -= 360;
        }
        return Rotation2d.fromDegrees(rot);        
    }
}
