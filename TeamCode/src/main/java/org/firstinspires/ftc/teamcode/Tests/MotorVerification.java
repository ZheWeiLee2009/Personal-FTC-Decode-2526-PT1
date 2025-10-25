package org.firstinspires.ftc.teamcode.Tests;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Config.Drivetrain;

@Disabled
@TeleOp(name = "MotorVerification", group = "TeleOp")
public class MotorVerification extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    Drivetrain bot;

    static final double SPEED_MULTIPLIER = 0.5;

    public void runOpMode() {
        //bot initialization
        bot = new Drivetrain(hardwareMap, runtime);
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.addLine("After 5 seconds, the front left motor " +
                "will run at full speed for 1 second.");

        telemetry.update();
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            sleep(1000);
            telemetry.addLine("left front drive");
            bot.leftFrontDrive.setPower(1.0);
            sleep(1000);
            bot.leftFrontDrive.setPower(0.0);

            telemetry.addLine("right front drive");
            sleep(1000);
            bot.rightFrontDrive.setPower(1.0);
            sleep(1000);
            bot.rightFrontDrive.setPower(0.0);

            telemetry.addLine("left back drive");
            sleep(1000);
            bot.leftBackDrive.setPower(1.0);
            sleep(1000);
            bot.leftBackDrive.setPower(0.0);

            telemetry.addLine("right back drive");
            sleep(1000);
            bot.rightBackDrive.setPower(1.0);
            sleep(1000);
            bot.rightBackDrive.setPower(0.0);

            telemetry.addLine("Drive complete");
            telemetry.update();

            sleep(5000);
            bot.Flywheel.setPower(1.0);
            sleep(1000);
            bot.Flywheel.setPower(0);
            sleep(1000);
            bot.Intake.setPower(0.4);
            sleep(1000);
            bot.Intake.setPower(0);

            telemetry.addLine("Aux complete");
            telemetry.update();

            FtcDashboard dashboard = FtcDashboard.getInstance();
            Telemetry dashboardTelemetry = dashboard.getTelemetry();

            dashboardTelemetry.addData("Flywheel Power: ", bot.Flywheel.getPower());
            dashboardTelemetry.update();


        }
    }
}