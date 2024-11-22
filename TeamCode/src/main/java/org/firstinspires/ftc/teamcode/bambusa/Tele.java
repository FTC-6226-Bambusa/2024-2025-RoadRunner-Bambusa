package org.firstinspires.ftc.teamcode.bambusa;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

@Config
@TeleOp
public class Tele extends LinearOpMode {
    MecanumDrive drive;
    Robot robot;

    public static double p = 0, i = 0, d = 0, f = 0;

    private DcMotorEx slideChain;

    public double tgt;

    @Override
    public void runOpMode() throws InterruptedException {

        // Robot & PID
        robot = new Robot(hardwareMap);
        robot.setPID(p, i, d, f);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // Variables
            int armPos = robot.slideChain.getCurrentPosition();
            int tgt = gamepad1.y ? 230 : 30;

            // Robot Functions
            robot.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_trigger);
            robot.extend(1, 50, gamepad1.dpad_up, gamepad1.dpad_down);
            robot.slideChainToPosition(tgt);

            // Telemetry
            telemetry.addData("Dpad Down? ", gamepad1.dpad_down);
            telemetry.addData("Dpad Up? ", gamepad1.dpad_up);
            telemetry.addData("Gamepad1 Y: ", gamepad1.y);

            telemetry.addData("Chain Pos: ", armPos);
            telemetry.addData("Target", tgt);

            telemetry.update();

        }
    }
}