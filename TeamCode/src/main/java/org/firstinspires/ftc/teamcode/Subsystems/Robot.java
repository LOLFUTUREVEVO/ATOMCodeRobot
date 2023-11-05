package org.firstinspires.ftc.teamcode.Subsystems;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

// This class defines the DriveTrain and its constraints for the 23806 ATOM ROBOTICS robot
public class Robot {
    public DcMotor LeftFront;
    public DcMotor RightFront;
    public DcMotor LeftBack;
    public DcMotor RightBack;


    public Robot(HardwareMap hw) {
        LeftFront = hw.get(DcMotor.class, "frontleft");
        RightFront = hw.get(DcMotor.class, "frontright");
        LeftBack = hw.get(DcMotor.class, "backleft");
        RightBack = hw.get(DcMotor.class, "backright");
        RightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        //RightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        LeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    public void setDrivePower(double y,double x, double rx) {
        LeftFront.setPower(y + x + rx);
        RightFront.setPower(y - x - rx);
        LeftBack.setPower(y - x + rx);
        RightBack.setPower(y + x - rx);
    }

    public int[] getEncVals(){
        return new int[] {LeftFront.getCurrentPosition(), RightFront.getCurrentPosition(), LeftBack.getCurrentPosition(), RightBack.getCurrentPosition()};
    }
    // For TeleOp Usage Only
    public void commandCenter(Gamepad gamepad1, Gamepad gamepad2) {

    }
}
