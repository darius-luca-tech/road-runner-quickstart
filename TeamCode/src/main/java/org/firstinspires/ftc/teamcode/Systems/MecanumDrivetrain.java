package org.firstinspires.ftc.teamcode.Systems;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


public class MecanumDrivetrain {
    //Variables
    public static double MAX_V = 10; //max velocity
    public static double MAX_O = 10; //angular velocity
    public double wheelOmega = 0; //angular frequency
    public static final double RADIUS = 2; //inch
    private final boolean zeroStrafeCorrection = false;

    //Motor Caching
    private double lastFRPower = 0;
    private double lastBRPower = 0;
    private double lastFLPower = 0;
    private double lastBLPower = 0;
    public final double motorUpdateTolerance = 0.05;

    public static double MOVE_FORWARD = -0.5; //power values
    public static double MOVE_BACK = 0.5;
    public static double MOVE_RIGHT = 0.5;
    public static double MOVE_LEFT = -0.5;
    public static double OmegaSpeed = 0.7;

    public static Vector2d[] WHEEL_POSITION = { //wheel distance from the center (in xOy/inch)
            new Vector2d(6, 7.5),
            new Vector2d(-6, 7.5),
            new Vector2d(-6, -7.5),
            new Vector2d(6, -7.5)
    };
    public static Vector2d[] ROTOR_DIRECTIONS = { //motor polarities (not sure)
            new Vector2d(1, 1),
            new Vector2d(-1, 1),
            new Vector2d(-1, -1),
            new Vector2d(1, -1)
    };
    private Pose2d targetVelocity = new Pose2d(0, 0, 0);

    //Motors
    public DcMotorEx[] driveTrainMotors = new DcMotorEx[4];
    //Sensors
    private BNO055IMU IMU;
    //Hub

    public MecanumDrivetrain(HardwareMap hardwareMap) {

        IMU = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters Parameters = new BNO055IMU.Parameters();
        Parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        IMU.initialize(Parameters);

        driveTrainMotors[0] = hardwareMap.get(DcMotorEx.class, "motorFL");
        driveTrainMotors[1] = hardwareMap.get(DcMotorEx.class, "motorFR");
        driveTrainMotors[2] = hardwareMap.get(DcMotorEx.class, "motorBL");
        driveTrainMotors[3] = hardwareMap.get(DcMotorEx.class, "motorBR");

        for(DcMotor i : driveTrainMotors) {
            i.setPower(0);
        }

        for(DcMotor i : driveTrainMotors) {
            i.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            i.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        driveTrainMotors[0].setDirection(DcMotorEx.Direction.REVERSE);
        driveTrainMotors[1].setDirection(DcMotorEx.Direction.FORWARD);
        driveTrainMotors[2].setDirection(DcMotorEx.Direction.FORWARD);
        driveTrainMotors[3].setDirection(DcMotorEx.Direction.REVERSE);

        }

        public void changeSpeed(int Speed) {
            if (Speed == 1) {
                MAX_V = 3;
                MAX_O = 2;
            } else if (Speed == 2) {
                MAX_V = 5;
                MAX_O = 3;
            }
        }

        public void setPower(Vector2d v, double omega) {
            setVelocity(v.times(MAX_V), omega * MAX_O);
        }

        public void setVelocity(Vector2d v, double omega) {
            for(int i = 0; i < 4; ++i) {
                Vector2d wheelVelocity = new Vector2d(v.getX() - omega * WHEEL_POSITION[i].getY(),
                                                        v.getY() + omega * WHEEL_POSITION[i].getX());
                wheelOmega = (wheelVelocity.dot(ROTOR_DIRECTIONS[i]) * Math.sqrt(2)) / RADIUS;
                driveTrainMotors[i].setVelocity(wheelOmega, AngleUnit.RADIANS);
            }
        }

        public void setPower(Pose2d target) {
            double v = target.vec().norm() * MAX_V;
            double theta = Math.atan2(target.getX(), target.getY());
            double omega = target.getHeading() * MAX_O;

            targetVelocity = new Pose2d(v * Math.cos(theta), v * Math.sin(theta), omega);
            setVelocity(targetVelocity);
        }

        public void setVelocity(Pose2d v) {
            for(int i = 0; i < 4; ++i) {
                Vector2d WheelVelocity = new Vector2d(v.getX() - v.getHeading() * WHEEL_POSITION[i].getY(),
                                                         v.getY() + v.getHeading() * WHEEL_POSITION[i].getX());
                wheelOmega = (WheelVelocity.dot(ROTOR_DIRECTIONS[i]) * Math.sqrt(2)) / RADIUS;
                driveTrainMotors[i].setVelocity(wheelOmega, AngleUnit.RADIANS);
            }
        }


        //Change polarities when testing teleop
        public void SlideRight(int power) {
            driveTrainMotors[0].setPower(-power);
            driveTrainMotors[1].setPower(-power);
            driveTrainMotors[2].setPower(power);
            driveTrainMotors[3].setPower(power);
        }

        public void SlideLeft(int power) {
            driveTrainMotors[0].setPower(power);
            driveTrainMotors[1].setPower(power);
            driveTrainMotors[2].setPower(-power);
            driveTrainMotors[3].setPower(-power);
        }

        public void MoveForward(int power) {
            driveTrainMotors[0].setPower(-power);
            driveTrainMotors[1].setPower(power);
            driveTrainMotors[2].setPower(power);
            driveTrainMotors[3].setPower(-power);
        }

        public void MoveBackward(int power) {
            driveTrainMotors[0].setPower(power);
            driveTrainMotors[1].setPower(-power);
            driveTrainMotors[2].setPower(-power);
            driveTrainMotors[3].setPower(power);
        }

        public void TurnLeft(int power) {
            driveTrainMotors[0].setPower(-power);
            driveTrainMotors[1].setPower(power);
            driveTrainMotors[2].setPower(-power);
            driveTrainMotors[3].setPower(power);
        }

        public void TurnRight(int power) {
            driveTrainMotors[0].setPower(power);
            driveTrainMotors[1].setPower(-power);
            driveTrainMotors[2].setPower(power);
            driveTrainMotors[3].setPower(-power);
        }

        public void setControls(double xdot, double ydot, double w) {
            double FLPower;
            double FRPower;
            double BLPower;
            double BRPower;

            if(!zeroStrafeCorrection) {
                FLPower = ydot + xdot + w;
                FRPower = ydot + xdot - w;
                BLPower = -ydot + xdot - w;
                BRPower = -ydot + xdot + w;
            } else {
                FLPower = xdot + w;
                BLPower = xdot - w;
                FRPower = xdot + w;
                BRPower = xdot + w;
            }

            double maxpower = Math.max(Math.abs(FRPower), Math.max(Math.abs(BLPower), Math.max(Math.abs(FLPower), Math.abs(BRPower))));

            if(maxpower > 1) {
                FRPower /= maxpower;
                BLPower /= maxpower;
                FLPower /= maxpower;
                BRPower /= maxpower;
            }

            if(xdot == 0 && ydot == 0 && w == 0) {
                driveTrainMotors[0].setPower(FRPower);
                driveTrainMotors[1].setPower(FLPower);
                driveTrainMotors[2].setPower(BLPower);
                driveTrainMotors[3].setPower(BRPower);

                //cache the motors power
                // to do: check the motor tolerance with the previous power
            }
        }
}

