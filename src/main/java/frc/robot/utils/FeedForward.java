/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package frc.robot.utils;

import frc.robot.Constants;

/**
 *
 * @author Udi Kislev
 */
public class FeedForward {
    
    public double K_H = 0.07;
    public double K_L = -0.0696;
    public double K_S_L = Constants.KS / 12; //0.22404
    public double K_S_R = Constants.KS / 12;
    public double K_V_L = Constants.KV / 12; //0.04314
    public double K_V_R = Constants.KV / 12;
    public double leftV;
    public double rightV;
    public double leftP;
    public double rightP;

   public void calculate(double left, double right) {
       leftV = left;
       rightV = right;
       leftP = feedForwardLeft(leftV);
       rightP = feedForwardRight(rightV);
       if(left * right < 0) {
           return;
       }
       double dv = left - right;

       if(Math.abs(left) > Math.abs(right)) {
           leftP += dv * K_H;
           rightP += dv * K_L;
       } else {
        leftP -= dv * K_L;
        rightP -= dv * K_H;
       }
    }
    
    private double feedForwardRight(double velocity) {
        return velocity * K_V_R + Math.signum(velocity) * K_S_R;
    }

    private double feedForwardLeft(double velocity) {
        return velocity * K_V_L + Math.signum(velocity) * K_S_L;
    }
}
