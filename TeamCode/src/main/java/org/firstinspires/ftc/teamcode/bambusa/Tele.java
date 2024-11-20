package org.firstinspires.ftc.teamcode.bambusa;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@TeleOp
public class Tele extends LinearOpMode {
    MecanumDrive drive;
    Robot robot;

    public static double p = 0, i = 0, d = 0, f = 0;

    private Chain_PID chainPID = new Chain_PID(p, i, d, f);

    private DcMotorEx slideChain;

    double tgt;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        slideChain = hardwareMap.get(DcMotorEx.class, "slideChain");
        //slideChain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            robot.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_trigger);
            robot.extend(gamepad1.dpad_up, gamepad1.dpad_down);

            int armPos = slideChain.getCurrentPosition();

            boolean ifY = gamepad1.y;
            int tgt = (ifY ? 230 : 30);

            slideChain.setPower(chainPID.PIDControl(tgt, armPos));

            telemetry.addData("Dpad down?", gamepad1.dpad_down);
            telemetry.addData("Dpad up?", gamepad1.dpad_up);
            //telemetry.addData("Chain position: ", robot.slideChain.getCurrentPosition());
            telemetry.addData("Gamepad1y?: ", gamepad1.y);
            //telemetry.addData("chain target position: ", robot.slideChain.getTargetPosition());

            telemetry.addData("pos: ", armPos);
            telemetry.addData("target", tgt);
            telemetry.update();




        }
    }
}