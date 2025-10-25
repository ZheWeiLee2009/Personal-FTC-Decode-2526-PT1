package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Config.Drivetrain;

import java.util.Locale;

@Disabled
@TeleOp(name = "Odo Tracker", group = "TeleOp")
public class OdoTracker extends OpMode {
    Drivetrain bot;

    private ElapsedTime opmodeTimer = new ElapsedTime();
    private double SPEED_MULTIPLIER = 0.9;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();


    @Override
    public void init() {
        opmodeTimer.reset();

        bot = new Drivetrain(hardwareMap, opmodeTimer);
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        bot.odo.resetPosAndIMU();

        telemetry.addData("Odo.Status", "Initialized");
        telemetry.addData("X offset", bot.odo.getXOffset(DistanceUnit.MM));
        telemetry.addData("Y offset", bot.odo.getYOffset(DistanceUnit.MM));
        telemetry.addData("Device Version Number:", bot.odo.getDeviceVersion());
        telemetry.addData("Heading Scalar", bot.odo.getYawScalar());

        telemetry.addData("Current Speed: ", SPEED_MULTIPLIER);
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.reset();
        bot.oldTime = 0;
        bot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop(){
        // Odo update
        bot.odo.update();

        double newTime = getRuntime();
        double loopTime = newTime - bot.oldTime;
        double frequency = 1/loopTime;
        bot.oldTime = newTime;

        // Directional Movements
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        double[] powers = bot.calculateMotorPowers(y,x,rx);
        bot.setMotorPowers(powers[0], powers[1], powers[2], powers[3], SPEED_MULTIPLIER);

        // telemetry
        telemetry.addData("\nFL: ", bot.leftFrontDrive.getPower());
        telemetry.addData("BL: ", bot.leftBackDrive.getPower());
        telemetry.addData("FR: ", bot.rightFrontDrive.getPower());
        telemetry.addData("BR: ", bot.rightBackDrive.getPower());

        telemetry.addData("\n\n Full Power: ", SPEED_MULTIPLIER);

        /*
        gets the current Position (x & y in mm, and heading in degrees) of the robot, and prints it.
        */
        Pose2D pos = bot.odo.getPosition();
        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Position", data);

        /*
        gets the current Velocity (x & y in mm/sec and heading in degrees/sec) and prints it.
        */
        String velocity = String.format(Locale.US,"{XVel: %.3f, YVel: %.3f, HVel: %.3f}", bot.odo.getVelX(DistanceUnit.MM), bot.odo.getVelY(DistanceUnit.MM), bot.odo.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));
        telemetry.addData("Velocity", velocity);

        telemetry.addData("Status", bot.odo.getDeviceStatus());
        telemetry.addData("Pinpoint Frequency", bot.odo.getFrequency()); //prints/gets the current refresh rate of the Pinpoint
        telemetry.addData("REV Hub Frequency: ", frequency); //prints the control system refresh rate
        telemetry.update();

        telemetry.update();
    }
}
