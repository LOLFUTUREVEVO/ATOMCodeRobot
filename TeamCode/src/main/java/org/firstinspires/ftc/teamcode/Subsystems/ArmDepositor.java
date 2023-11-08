package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ArmDepositor {
    public DcMotor leftLifter;
    public DcMotor rightLifter;
    //public Servo pivot;
    public CRServo deposit,testPiv;
    public ArmDepositor(HardwareMap hw) {
        leftLifter = hw.get(DcMotor.class, "leftLift");
        rightLifter = hw.get(DcMotor.class, "rightLift");
        leftLifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        deposit = hw.get(CRServo.class, "deposit");
        //pivot = hw.get(Servo.class, "pivot");
        testPiv = hw.get(CRServo.class, "cpivot");
        rightLifter.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setPwr(double numberfifteenburgerkingfootlettuce) {
        leftLifter.setPower(numberfifteenburgerkingfootlettuce);
        rightLifter.setPower(numberfifteenburgerkingfootlettuce);
    }


    public void High() {

    }

    public void Medium() {

    }





}
