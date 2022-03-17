// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import frc.robot.commands.InstantCommandInDisable;
import frc.robot.commands.LedTransition;
import frc.robot.commands.MoveBetweenColors;
import frc.robot.commands.Rainbow;

/** Add your docs here. */
public final class LedHandler {
    private static final AddressableLED led = new AddressableLED(Constants.LED1_PORT);
    // private static final AddressableLED led2 = new AddressableLED(Constants.LED2_PORT);
    private static final AddressableLEDBuffer buffer = new AddressableLEDBuffer(Constants.LED_COUNT);
    private static Thread thread = new Thread();

    public static void init() {
        led.setLength(Constants.LED_COUNT);
        // led2.setLength(Constants.LED_COUNT);
        setColor(135, 45, 255);
        led.start();
        // led2.start();
        SmartDashboard.putNumber("Leds/Duration", 0);
        SmartDashboard.putData("Leds/start", 
        new LedTransition(() -> {
            return bgrToHsv(new Color(SmartDashboard.getNumber("Leds/Red", 0) / 255,
                    SmartDashboard.getNumber("Leds/Green", 0) / 255,
                    SmartDashboard.getNumber("Leds/Blue", 0) / 255));
                },
                    () -> {
                        return SmartDashboard.getNumber("Leds/Duration", 0);
                    }
        ));
        SmartDashboard.putData("Leds/Set",
        new InstantCommandInDisable(() -> {
            setColor(SmartDashboard.getNumber("Leds/Red", 0),
                    SmartDashboard.getNumber("Leds/Green", 0),
                    SmartDashboard.getNumber("Leds/Blue", 0));
        }));
        SmartDashboard.putData("Leds/Rainbow", new Rainbow());

        SmartDashboard.putData("Leds/Test", NetworkTableInstance.getDefault().getEntry("FMSInfo/IsRedAlliance").getBoolean(true) ? new MoveBetweenColors(142, 180) : new MoveBetweenColors(120, 133));
    }

    public static void setColor(int index, int red, int green, int blue) {
        stopCurrentTransition();
        buffer.setRGB(index, red, green, blue);
        led.setData(buffer);
        // led2.setData(buffer);
    }

    public static void setColor(double red, double green, double blue) {
        stopCurrentTransition();
        for (int i = 0; i < Constants.LED_COUNT; i++) {
            buffer.setRGB(i, (int)red, (int)green, (int)blue);
        }
        led.setData(buffer);
        // led2.setData(buffer);
        updateSmartDashboard(red, green, blue);
    }
    
    public static void setColorWithOffset(double h, double s, double v, double offset) {
        for (int i = 0; i < Constants.LED_COUNT; i++) {
            buffer.setHSV(i, (int) ((h + i * offset) % 180), (int) s, (int) v);
        }
        led.setData(buffer);
        // led2.setData(buffer);
    }

    public static void setColorWithOffsetAndRoofs(double h, double s, double v, double offset, double max, double min) {
        for (int i = 0; i < Constants.LED_COUNT; i++) {
            buffer.setHSV(i, (int) Math.min(Math.max((h + i * offset), min), max) % 180, (int) s, (int) v);
        }
        led.setData(buffer);
        // led2.setData(buffer);
    }

    public static void setHsv(int index, double hue, double saturation, double value) {
        stopCurrentTransition();
        buffer.setHSV(index, (int) hue, (int) saturation, (int) value);
        led.setData(buffer);
        // led2.setData(buffer);
    }

    public static void setHsv(double hue, double saturation, double value) {
        stopCurrentTransition();
        for (int i = 0; i < Constants.LED_COUNT; i++) {
            buffer.setHSV(i, (int) hue, (int) saturation, (int) value);
        }
        led.setData(buffer);
        // led2.setData(buffer);
        updateSmartDashboard(buffer.getLED(0).red * 255, buffer.getLED(0).green * 255, buffer.getLED(0).blue * 255);
    }

    public static void transitionToBgr(double red, double green, double blue, double duration) {
        double[] hsv = bgrToHsv(new Color(red, green, blue));
        transitionTo(hsv[0], hsv[1], hsv[2], duration);
    }

    public static double[] getHsv() {
        return bgrToHsv(buffer.getLED(0));
    }

    public static void transitionTo(double hue, double saturation, double value, double duration) {
        double millis = duration * 1000;
        stopCurrentTransition();
        thread = new Thread(() -> {
            double[] hsv = bgrToHsv(buffer.getLED(0));
            double startHue = hsv[0];
            double startSaturation = hsv[1];
            double startValue = hsv[2];
            double endHue = hue;
            double endSaturation = saturation;
            double endValue = value;
            int step = 0;
            while (step < millis) {
                double currentHue = (startHue + (endHue - startHue) * step / millis);
                double currentSaturation = (startSaturation + (endSaturation - startSaturation) * step / millis);
                double currentValue = (startValue + (endValue - startValue) * step / millis);
                setHsv(currentHue, currentSaturation, currentValue);
                step += 100;
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            setHsv(hue, saturation, value);
        });
        thread.start();
    }

    public static double[] bgrToHsv(Color color) {
        double[] hsv = new double[3];
        double red = color.red;
        double green = color.green;
        double blue = color.blue;
        double max = Math.max(red, Math.max(green, blue));
        double min = Math.min(red, Math.min(green, blue));
        double delta = max - min;
        hsv[2] = max;
        if (max != 0) {
            hsv[1] = delta / max;
        } else {
            hsv[1] = 0;
            hsv[0] = -1;
            return hsv;
        }
        if (red == max) {
            hsv[0] = (green - blue) / delta;
        } else if (green == max) {
            hsv[0] = 2 + (blue - red) / delta;
        } else {
            hsv[0] = 4 + (red - green) / delta;
        }
        hsv[0] *= 60;
        if (hsv[0] < 0) {
            hsv[0] += 360;
        }
        hsv[0] *= 0.5;
        hsv[1] *= 255;
        hsv[2] *= 255;
        return hsv;
    }

    public static void stopCurrentTransition() {
        if (thread.isAlive()) {
            thread.interrupt();
        }
    }

    private static void updateSmartDashboard(double red, double green, double blue) {
        SmartDashboard.putNumber("Leds/Red", red);
        SmartDashboard.putNumber("Leds/Green", green);
        SmartDashboard.putNumber("Leds/Blue", blue);
    }
}
