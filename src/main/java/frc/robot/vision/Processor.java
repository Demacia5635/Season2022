// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import java.util.ArrayList;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.Constants;
import frc.robot.utils.VisionProcessor;

/** Add your docs here. */
public class Processor extends VisionProcessor{

    private static final Mat kernel = Mat.ones(Constants.KERNEL_SIZE, Constants.KERNEL_SIZE, CvType.CV_32F);

    public Processor(int cameraWidth, int cameraHeight){
        super(cameraWidth, cameraHeight);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Vision Distance", () -> {return get("distance");}, null);
        builder.addDoubleProperty("Vision Angle", () -> {return get("angle");}, null);
    }

    /**
     * calculates the distance to the object by it's y value on the camera
     * @param y the y value on the camera
     * @return the distance to the object, in meters
     */
    private double distance(int y){
        return (Constants.TOWER_HEIGHT - Constants.CAMERA_HEIGHT) /
                Math.tan(Math.toRadians((Constants.CAMERA_FOV_HEIGHT *
                (Constants.CAMERA_HEIGHT_PIXELS - y) /
                Constants.CAMERA_HEIGHT_PIXELS) + Constants.CAMERA_ANGLE));
    }

    /**
     * calculates the angle to the object by it's x value on the camera
     * @param x the x value on the camera
     * @return the angle to the object, in radians
     */
    private double angle(int x){
        return Math.toRadians((x - Constants.CAMERA_WIDTH_PIXELS / 2) *
                Constants.CAMERA_FOV_WIDTH / Constants.CAMERA_WIDTH_PIXELS);
    }

    /**
     * finds the tower relative to the camera
     * @param a1 the x value of the first point
     * @param b1 the y value of the first point
     * @param a2 the x value of the second point
     * @param b2 the y value of the second point
     * @return the tower's {x, y}
     */
    private double[] findTower(double a1, double b1, double a2, double b2){
        if (b1 == b2){
            return new double[]{(a1 + a2) / 2, b1 + Math.sqrt(Constants.TOWER_RADIUS * Constants.TOWER_RADIUS - 
                    (a1 + a2) * (a1 + a2) / 4)};
        }

        double m = (a2 - a1) / (b1 - b2);
        double b = (b1 * b1 - b2 * b2 + a1 * a1 - a2 * a2) / (2 * (b1 - b2));
        double t = (2 * a1 - 2 * m * b + 2 * b1 + Math.signum(m) * Math.sqrt(
                (2 * a1 - 2 * m * b + 2 * m * b1) * (2 * a1 - 2 * m * b + 2 * m * b1) -
                4 * (m * m + 1) * (a1 * a1 + b * b - 2 * b * b1 + b1 * b1 -
                Constants.TOWER_RADIUS * Constants.TOWER_RADIUS)
                )) / (2 * (m * m + 1));
        return new double[]{t, t * m + b};
    }

    @Override
    public void calculate(Mat frame) {
        Imgproc.cvtColor(frame, frame, Imgproc.COLOR_BGR2GRAY);
        Core.inRange(frame, new Scalar(Constants.MIN_GRAYSCALE_VISION), new Scalar(Constants.MAX_GRAYSCALE_VISION), frame);
        ArrayList<MatOfPoint> contours = new ArrayList<MatOfPoint>();

        Imgproc.erode(frame, frame, kernel);
        Imgproc.dilate(frame, frame, kernel);

        Imgproc.findContours(frame, contours, null, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        int size = contours.size();
        if (size <= 1) {
            put("distance", Double.NaN);
            put("angle", Double.NaN);
            return;
        }

        double[] lastPoint = null;
        double avarageDistance = 0;
        double avarageAngle = 0;
        double amount = 0;

        for (MatOfPoint contour : contours){
            if (Imgproc.contourArea(contour) > Constants.MIN_CONTOUR_SIZE) continue;
            int[] highestPointIndex = {0, 0};
            for (int i = 0; i < contour.size().height; i++){
                for (int j = 0; j < contour.size(i); j++){
                    if (contour.get(i, j)[1] > contour.get(highestPointIndex[0], highestPointIndex[1])[1]){
                        highestPointIndex[0] = i;
                        highestPointIndex[1] = j;
                    }
                }
            }
            double distance = distance((int)contour.get(highestPointIndex[0], highestPointIndex[1])[1]);
            double angle = angle((int)contour.get(highestPointIndex[0], highestPointIndex[1])[0]);

            double[] currentPoint = {distance * Math.sin(angle), distance * Math.cos(angle)};

            if (lastPoint != null){
                double[] towerPoint = findTower(currentPoint[0], currentPoint[1], lastPoint[0], lastPoint[1]);
                double distanceToTower = Math.sqrt(towerPoint[0] * towerPoint[0] + towerPoint[1] * towerPoint[1]);
                double angleToTower =  Math.atan(towerPoint[1] / towerPoint[0]);
                avarageDistance = (amount * avarageDistance + distanceToTower) / (amount + 1);
                avarageAngle = (amount * avarageAngle + angleToTower) / (amount + 1);
                amount++;
            }
            lastPoint = currentPoint;
        }

        put("distance", avarageDistance);
        put("angle", avarageAngle);
    }
}
