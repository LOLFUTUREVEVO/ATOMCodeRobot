package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.ArmDepositor;
import org.firstinspires.ftc.teamcode.Subsystems.Climber;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Utilities.PIDController;

@Config  // This allows for usage of the dashboard
@TeleOp(name="Hellow Jawa")
public class teleOp extends LinearOpMode {

    public Robot rb;
    public ArmDepositor arm;
    public Climber cl;
    public PIDController pid;
    public DcMotor shooter;


    @Override
    public void runOpMode() throws InterruptedException {

        rb = new Robot(hardwareMap);
        cl = new Climber(hardwareMap);
        arm = new ArmDepositor(hardwareMap);
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        FtcDashboard dashboard =FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        waitForStart();
        // Start code (runs before the main loop)


        while(!isStopRequested()) {
            // Main Loop

            rb.setDrivePower(gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x*0.8);

            if(gamepad1.dpad_up) {
                cl.setPwr(1);
            } else if(gamepad1.dpad_down) {
                cl.setPwr(-1);
            } else {
                cl.setPwr(0.0);
            }//

            if(gamepad1.back){
                arm.leftLifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                arm.rightLifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                arm.leftLifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                arm.rightLifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            //ATEF IS SMOKING CRACK

            if(gamepad1.right_bumper) {
                arm.testPiv.setPower(0.7);
            }else if(gamepad1.left_bumper) {
                arm.testPiv.setPower(-0.7);
            }else {
                arm.testPiv.setPower(0);
            }


            if(gamepad1.a) {
                arm.deposit.setPower(1);
            }else if(gamepad1.x) {
                arm.deposit.setPower(0);
            }else if(gamepad1.b) {
                arm.deposit.setPower(-0.3);
            }

            if(gamepad1.y) {
                shooter.setPower(-1);
            }else {
                shooter.setPower(0);
            }



            arm.setPwr(gamepad1.right_trigger-gamepad1.left_trigger);

            dashboardTelemetry.addData("right", arm.rightLifter.getCurrentPosition());
            dashboardTelemetry.addData("left", arm.leftLifter.getCurrentPosition());
            dashboardTelemetry.update();
            
        }
    }
}
