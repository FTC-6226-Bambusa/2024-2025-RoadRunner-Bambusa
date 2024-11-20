package org.firstinspires.ftc.teamcode.bambusa;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous
public class Auto extends LinearOpMode {

    public enum Stage {
        STAGE1,
        STAGE2,
        IDLE;
    }

    ElapsedTime timer = new ElapsedTime();

    Stage stage = Stage.STAGE1;

    MecanumDrive drive;
    Robot robot;

    private PIDController controller;

    public static double p = 0.01, i = 0.2, d = 0.0011;
    public static double f = 0.5;

    public static int target = 30;

    private final double ticks_in_degree = 300 / 90.0;

    private DcMotorEx slideChain;

    double tgt;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot(hardwareMap);
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        slideChain = hardwareMap.get(DcMotorEx.class, "slideChain");
        slideChain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {

            switch (stage) {
                case STAGE1:

                    robot.drive(0, 1, 0, 0.2);

                    if (timer.milliseconds() > 0.5 * 1000) {
                        stage = Stage.STAGE2;
                        timer.reset();
                    }

                    break;
                case STAGE2:

                    robot.drive(0, 1, 0, 0.2);

                    if (timer.milliseconds() > 0.5 * 1000) {
                        stage = Stage.IDLE;
                        timer.reset();
                    }

                    break;

                case IDLE:
                    // Do Nothing :)
                    robot.drive(0, 0, 0, 0);
                    break;
            }

        }
    }
}
