package org.firstinspires.ftc.teamcode;

public class PIDController {
    double P;
    double I;
    double D;
    private double lastError = 0;
    private double totalError = 0;

    public PIDController (double P, double I, double D) {
        this.P = P;
        this.I = I;
        this.D = D;
    }

    public double update (double error) {
        double output = 0;
        output += P * error;
        totalError += error;
        output += I * totalError;
        output += D * (error - lastError);
        lastError = error;

        return output;
    }
}
