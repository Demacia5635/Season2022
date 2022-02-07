package frc.robot.utils;
public class PID{
    private double KP;
    private double KI;
    private double KD;
    private double last_error;
    private double sum_error;
    private double error;
    private double PID;
    private double setPoint;
    /**
     * Constructor for PID
     * @param KP
     * @param KI
     * @param KD
     */
    public PID(double KP, double KI, double KD){
        this.KP = KP;
        this.KI = KI;
        this.KD = KD;
    }

    /**
     * Sets destination for PID and resets the attributes
     * @param setPoint
     */
    public void setPoint(double setPoint){
        this.setPoint = setPoint;
        sum_error = 0;
        last_error = 0;
        error = 0;
    }

    /**
     * Calculates the error 
     * @param setPointSum The starting point + the destination
     * @param CurrentValue The Starting point
     * @return PID between 1 to -1 
     */
    public double calculate(double current){
        error = setPoint - current;
        sum_error += error;
        PID = (error * KP) + (sum_error * KI) + ((last_error-error)*KD);
        last_error = error;
        return Math.min(Math.max(PID,-1),1);
    }
}

