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
    static final double COUNTS_PER_INCH = 537.6;

    public Robot(HardwareMap hw) {
        LeftFront = hw.get(DcMotor.class, "frontleft");
        RightFront = hw.get(DcMotor.class, "frontright");
        LeftBack = hw.get(DcMotor.class, "backleft");
        RightBack = hw.get(DcMotor.class, "backright");
        RightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        RightFront.setDirection(DcMotorSimple.Direction.REVERSE);
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



    public void moveInches(double frontLeftInches, double frontRightInches, double backLeftInches, double backRightInches) {
        int frontLeftTarget = (int) (frontLeftInches * COUNTS_PER_INCH);
        int frontRightTarget = (int) (frontRightInches * COUNTS_PER_INCH);
        int backLeftTarget = (int) (backLeftInches * COUNTS_PER_INCH);
        int backRightTarget = (int) (backRightInches * COUNTS_PER_INCH);

        // Reset encoders and start motion
        LeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftFront.setTargetPosition(frontLeftTarget);
        RightFront.setTargetPosition(frontRightTarget);
        LeftBack.setTargetPosition(backLeftTarget);
        RightBack.setTargetPosition(backRightTarget);

        LeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set power to the motors (adjust as needed)
        double power = 1;
        LeftFront.setPower(power);
        RightFront.setPower(power);
        LeftBack.setPower(power);
        RightBack.setPower(power);

        // Wait for motors to reach target position
        while (LeftFront.isBusy() && RightFront.isBusy() && LeftBack.isBusy() && RightBack.isBusy()) {
            // Do nothing - just wait
        }

        // Stop motors
        LeftFront.setPower(0);
        RightFront.setPower(0);
        LeftBack.setPower(0);
        RightBack.setPower(0);

        // Turn off RUN_TO_POSITION mode
        LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }





    public void commandCenter(Gamepad gamepad1, Gamepad gamepad2) {

    }
}
