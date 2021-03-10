package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Systems.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.Systems.SampleMecanumDrive;

@TeleOp(name = "TeleOp_V1")
public class TeleOp_V1 extends LinearOpMode {
    private boolean RobotCentric = true;
    private double XYSpeed = 1;
    private double thetaSpeed = 1;
    @Override
    public void runOpMode() {
        MecanumDrivetrain DriveTrain = new MecanumDrivetrain(hardwareMap);
        JoystickTransform JTransform = new JoystickTransform();

        /*
            INIT SCOPE

         */

        telemetry.addData("Initialized.....", "0");

        waitForStart();
        while(opModeIsActive()) {
            if(RobotCentric) {
                DriveTrain.setControls(-gamepad1.left_stick_y * XYSpeed, -gamepad1.left_stick_x * XYSpeed, -gamepad1.right_stick_x * thetaSpeed);
            }
        }


    }
}
