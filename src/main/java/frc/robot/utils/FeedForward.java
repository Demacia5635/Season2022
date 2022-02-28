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
    
    public double K_H = 0.755183876280745;
    public double K_L = 0.059717813093936;
    public double K_S = Constants.KS; //0.22404
    public double K_V = Constants.KV; //0.04314
    public double leftV;
    public double rightV;
    public double leftP;
    public double rightP;

   public void calculate(double left, double right) {
       leftV = left;
       rightV = right;
       leftP = feedForwardPower(leftV);
       rightP = feedForwardPower(rightV);
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
    
    private double feedForwardPower(double velocity) {
        return velocity * K_V + K_S;
    }
}