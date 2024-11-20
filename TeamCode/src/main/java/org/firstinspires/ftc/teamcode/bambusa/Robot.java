package org.firstinspires.ftc.teamcode.bambusa;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;

    DcMotor slideChain;
    DcMotor slideLeft;
    DcMotor slideRight;

    public double kP = 0;
    public double kI = 0;
    public double kD = 0;
    PIDController pid = new PIDController(kP, kI, kD);

    public double driveSpeed = 0.5;
    public double boostSpeed = 0.8;

    // Finds Value Of Point A To Point B
    private double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }

    public Robot(HardwareMap hardwareMap) {
        // Drive Train Motors
        this.frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        this.backLeftMotor = hardwareMap.dcMotor.get("leftRear");
        this.frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        this.backRightMotor = hardwareMap.dcMotor.get("rightRear");

        // Arm Motors
        this.slideChain = hardwareMap.dcMotor.get("slideChain");
        this.slideRight = hardwareMap.dcMotor.get("slideRight");
        this.slideLeft = hardwareMap.dcMotor.get("slideLeft");

        // Setting Motor Direction
        this.frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.slideLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        //Set Motor behaviors
        this.slideChain.setTargetPosition(0);
        this.slideChain.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // Modified GM0 Drive Code
    public void drive(double leftx, double lefty, double rightx, double lt) {
        double y = -lefty;
        double x = leftx * 1.1;
        double rx = rightx;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double speedMultiplier = lerp(driveSpeed, boostSpeed, lt);

        double frontLeftPower = (y + x + rx) / denominator * speedMultiplier;
        double backLeftPower = (y - x + rx) / denominator * speedMultiplier;
        double frontRightPower = (y - x - rx) / denominator * speedMultiplier;
        double backRightPower = (y + x - rx) / denominator * speedMultiplier;

        this.frontLeftMotor.setPower(frontLeftPower);
        this.backLeftMotor.setPower(backLeftPower);
        this.frontRightMotor.setPower(frontRightPower);
        this.backRightMotor.setPower(backRightPower);
    }

    // Extends Robot Arm
    public void extend(boolean dpadup, boolean dpaddown) {
        this.slideRight.setPower(dpadup ? 1 : (dpaddown ? -1 : 0));
        this.slideLeft.setPower(dpadup ? 1 : (dpaddown ? -1 : 0));
    }


    // Rotates Robot Arm To Position
   // public void slideToPosition(boolean ifY) {
       // PID_Arm.target = (ifY ? 230 : 30);


}
