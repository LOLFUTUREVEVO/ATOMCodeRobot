package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Climber {
    public DcMotor Cmotor;

    public Climber(HardwareMap hw) {
        Cmotor = hw.get(DcMotor.class, "cmotor");
    }


    public void setPwr(double p) {
        Cmotor.setPower(p);
    }

    public void beginClimb() {

    }

    public void endClimb() {

    }

}
