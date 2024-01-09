package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.ArmDepositor;
import org.firstinspires.ftc.teamcode.Subsystems.Climber;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;

@Config
@Autonomous(name = "leftRed")
public class ATOMQualOneAuto extends OpMode {


    public Robot dt;
    public Climber c;
    public ArmDepositor arm;

    @Override
    public void init() {
        dt = new Robot(hardwareMap);
        c = new Climber(hardwareMap);
        arm = new ArmDepositor(hardwareMap);

    }


    boolean s1=false,s2=false;


    @Override
    public void start() {
        dt.moveInches(-3.75, -3.75, -3.75,-3.75);
        dt.moveInches(-1.6, 1.61, -1.61, 1.61);
        dt.moveInches(-5, -5, -5, -5);
        dt.moveInches(-1,1,1,-1);
        dt.moveInches(-2.5,-2.5,-2.5,-2.5);
    }

    @Override
    public void loop() {


        ElapsedTime PidTimer = new ElapsedTime();

    }

    @Override
    public void stop() {
        telemetry.addData("OpMode Stopping","Auto Ending");
        telemetry.addData("Readying","TeleOp");
        telemetry.update();
    }




}
