// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.InstantCommandInDisable;
import frc.robot.commands.LedTransition;
import frc.robot.commands.MoveBetweenColors;
import frc.robot.commands.Rainbow;
import frc.robot.commands.StartEndCommandOnDisable;
 
public class LedHandler extends SubsystemBase {
  private final AddressableLED led;
  private final AddressableLEDBuffer buffer;

  public LedHandler() {
    led = new AddressableLED(Constants.LED_PORT);
    buffer = new AddressableLEDBuffer(Constants.LED_COUNT);
    led.setLength(Constants.LED_COUNT);
    led.start();
    setDefaultColor();

    setDefaultCommand(NetworkTableInstance.getDefault().getEntry("FMSInfo/IsRedAlliance").getBoolean(true) ?
        new MoveBetweenColors(120, 142, this) :
        new MoveBetweenColors(142, 180, this));
  }

  public void setColor(int index, int red, int green, int blue) {
    buffer.setRGB(Math.min(index, Constants.LED_COUNT / 2 - 1), red, green, blue);
    buffer.setRGB(Constants.LED_COUNT - 1 - index, red, green, blue);
    updateLeds();
  }

  public void setDefaultColor() {
    setColor(168, 0, 230);
  }

  public void setColor(double red, double green, double blue) {
      for (int i = 0; i < Constants.LED_COUNT; i++) {
          buffer.setRGB(i, (int)red, (int)green, (int)blue);
      }
      updateLeds();
      updateSmartDashboard(red, green, blue);
  }

  public void setColorWithOffset(double h, double s, double v, double offset) {
      for (int i = 0; i < Constants.LED_COUNT / 2; i++) {
          double hue = (h + i * offset) % 180;
          buffer.setHSV(i, (int) hue, (int) s, (int) v);
          buffer.setHSV(Constants.LED_COUNT - 1 - i, (int) hue, (int) s, (int) v);
      }
      updateLeds();
  }

  public void setColorWithOffsetAndRoofs(double h, double s, double v, double offset, double max, double min) {
      for (int i = 0; i < Constants.LED_COUNT / 2; i++) {
          double hue = Math.min(Math.max((h + i * offset), min), max) % 180;
          buffer.setHSV(i, (int) hue, (int) s, (int) v);
          buffer.setHSV(Constants.LED_COUNT - 1 - i, (int) hue, (int) s, (int) v);
      }
      updateLeds();
  }

  public void setHsv(int index, double hue, double saturation, double value) {
      buffer.setHSV(index, (int) hue, (int) saturation, (int) value);
      buffer.setHSV(Constants.LED_COUNT - 1 - index, (int) hue, (int) saturation, (int) value);
      updateLeds();
  }

  public void setHsv(double hue, double saturation, double value) {
      for (int i = 0; i < Constants.LED_COUNT; i++) {
          buffer.setHSV(i, (int) hue, (int) saturation, (int) value);
      }
      updateLeds();
      updateSmartDashboard(buffer.getLED(0).red * 255, buffer.getLED(0).green * 255, buffer.getLED(0).blue * 255);
  }

  public void setRangeHSV(int lower, int upper, double hue, double saturation, double value) {
      for (int i = lower; i < upper; i++) {
          buffer.setHSV(i, (int) hue, (int) saturation, (int) value);
          buffer.setHSV(Constants.LED_COUNT - 1 - i, (int) hue, (int) saturation, (int) value);
      }
      updateLeds();
  }

  public void setRangeColor(int lower, int upper, double red, double green, double blue) {
    for (int i = lower; i < upper; i++) {
        buffer.setRGB(i, (int) red, (int) green, (int) blue);
        buffer.setRGB(Constants.LED_COUNT - 1 - i, (int) red, (int) green, (int) blue);
    }
    updateLeds();
}

  public double[] getHsv() {
      return bgrToHsv(buffer.getLED(0));
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

  private void updateLeds() {
    led.setData(buffer);
  }

  private void updateSmartDashboard(double red, double green, double blue) {
      SmartDashboard.putNumber("Leds/Red", red);
      SmartDashboard.putNumber("Leds/Green", green);
      SmartDashboard.putNumber("Leds/Blue", blue);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
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
        , this));
        SmartDashboard.putData("Leds/Set",
        new InstantCommandInDisable(() -> {
            setColor(SmartDashboard.getNumber("Leds/Red", 0),
                SmartDashboard.getNumber("Leds/Green", 0),
                SmartDashboard.getNumber("Leds/Blue", 0));
        }));
        SmartDashboard.putData("Leds/Rainbow", new Rainbow(this));

        SmartDashboard.putData("Leds/Default", new StartEndCommandOnDisable(this::setDefaultColor, () -> {}, this));
  }
}
