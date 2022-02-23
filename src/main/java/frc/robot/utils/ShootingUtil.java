package frc.robot.utils;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class ShootingUtil {

    public static final Translation2d TargetLocation = new Translation2d(8.0,4.0);

    public static final double xToDistance[][] = {
        { -10, 1},
        { -9, 1.3},
        { -8, 1.7},
        { -5, 2.1},
        { -3, 2.7},
        { -1, 3},
        { 0, 3.5},
        { 1, 4},
        { 3, 4.1},
        { 5, 4.5},
        { 8, 5},
        { 10, 6}
    };

    public static final double distanceToVelocityAndAngle[][] = {
        { 1, 12, 80},
        { 2, 13, 77},
        { 3, 13.7,75},
        { 4, 14,73},
        { 5, 15,70},
        { 6, 17, 68},
        { 7, 18, 62},
        { 8, 19,57}
    };
    
    public static double VisionXtoDistance(double x) {
        int idx;
        for(idx = 1; idx < xToDistance.length && xToDistance[idx][0] < x; idx++);
        if(idx == xToDistance.length) {
            return xToDistance[idx-1][1];
        }
        double x1 = xToDistance[idx-1][0];
        double x2 = xToDistance[idx][0];
        double v1 = xToDistance[idx-1][1];
        double v2 = xToDistance[idx][1];
        double r = (x-x1)/(x2-x1);
        return r*(v2-v1) + v1;
    }
    public static double distanceToVelocity(double distance) {
        int idx;
        for(idx = 1; idx < distanceToVelocityAndAngle.length && distanceToVelocityAndAngle[idx][0] < distance; idx++);
        if(idx == distanceToVelocityAndAngle.length) {
            return distanceToVelocityAndAngle[idx-1][1];
        }
        double d1 = distanceToVelocityAndAngle[idx-1][0];
        double d2 = distanceToVelocityAndAngle[idx][0];
        double v1 = distanceToVelocityAndAngle[idx-1][1];
        double v2 = distanceToVelocityAndAngle[idx][1];
        double r = (distance-d1)/(d2-d1);
        return r*(v2-v1) + v1;
    }
    public static double distanceToAngle(double distance) {
        int idx;
        for(idx = 1; idx < distanceToVelocityAndAngle.length && distanceToVelocityAndAngle[idx][0] < distance; idx++);
        if(idx == distanceToVelocityAndAngle.length) {
            return distanceToVelocityAndAngle[idx-1][2];
        }
        double d1 = distanceToVelocityAndAngle[idx-1][0];
        double d2 = distanceToVelocityAndAngle[idx][0];
        double v1 = distanceToVelocityAndAngle[idx-1][2];
        double v2 = distanceToVelocityAndAngle[idx][2];
        double r = (distance-d1)/(d2-d1);
        return r*(v2-v1) + v1;
    }

    public static Translation2d getTargetPosition(Pose2d pose) {
        Translation2d t = TargetLocation.minus(pose.getTranslation());
        return t;
    }

    public static double getDistance(Translation2d t) {
        return t.getNorm();
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
