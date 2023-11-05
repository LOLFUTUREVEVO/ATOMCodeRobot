package org.firstinspires.ftc.teamcode.Utilities;

import com.qualcomm.robotcore.hardware.DcMotor;

public class PIDController {

    int reference, tolerance;
    double error=0;


    double Ki,Kp,Kd;
    public DcMotor runMotor;

    public PIDController(DcMotor i, double ki, double kp, double kd) {
        runMotor = i;
        Ki = ki;
        Kp = kp;
        Kd = kd;
    }

    public double calcErrSetPwr(int reference, int tolerance, double time, double LastError) {
        this.reference = reference;
        this.tolerance = tolerance;
        error = reference - runMotor.getCurrentPosition();
        double i = 0;
        if(Math.abs(error) > tolerance) {
            error = reference - runMotor.getCurrentPosition();
            double d = (error - LastError) / time;
            i = i + (error * time);
            double out = (Kp * error) +(Ki * i) +(Kd * d);
            runMotor.setPower(out);
            return error;
        }
        return 0;
    }

}
