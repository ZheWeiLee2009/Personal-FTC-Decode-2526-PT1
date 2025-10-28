package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Config.RobotConstants.c_BL_WeightTuning;
import static org.firstinspires.ftc.teamcode.Config.RobotConstants.c_BR_WeightTuning;
import static org.firstinspires.ftc.teamcode.Config.RobotConstants.c_FL_WeightTuning;
import static org.firstinspires.ftc.teamcode.Config.RobotConstants.c_FR_WeightTuning;
import static org.firstinspires.ftc.teamcode.Config.RobotConstants.initPower;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Config.Drivetrain;

@Disabled // Disable When done testing
@TeleOp(name = "MecanumWheelsTest", group = "Tests")
public class MechWheelTest extends OpMode {
    Drivetrain bot;

    private ElapsedTime runtime = new ElapsedTime();
    private double SPEED_MULTIPLIER = initPower;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();


    private double[] powers = new double[4];
    private String movementDireciton;

    @Override
    public void init() {
        runtime.reset();

        bot = new Drivetrain(hardwareMap, runtime);

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
    }


    @Override
    public void loop(){
        if (gamepad1.dpad_up){
            // Forward
            powers = bot.calculateMotorPowers(1,0,0);
            bot.setMotorPowers(powers[0], powers[1], powers[2], powers[3], SPEED_MULTIPLIER);
            movementDireciton = "Forward";

        } else if (gamepad1.dpad_right){
            // Side Right
            powers = bot.calculateMotorPowers(0,1,0);
            bot.setMotorPowers(powers[0], powers[1], powers[2], powers[3], SPEED_MULTIPLIER);
            movementDireciton = "Right";

        } else if (gamepad1.dpad_down){
            // Backwards
            powers = bot.calculateMotorPowers(-1,0,0);
            bot.setMotorPowers(powers[0], powers[1], powers[2], powers[3], SPEED_MULTIPLIER);
            movementDireciton = "Backwards";

        } else if (gamepad1.dpad_left){
            // Side Left
            powers = bot.calculateMotorPowers(0,-1,0);
            bot.setMotorPowers(powers[0], powers[1], powers[2], powers[3], SPEED_MULTIPLIER);
            movementDireciton = "Left";

        } else if (gamepad1.square){
            // Rotate Left
            powers = bot.calculateMotorPowers(0,0,1);
            bot.setMotorPowers(powers[0], powers[1], powers[2], powers[3], SPEED_MULTIPLIER);
            movementDireciton = "Rotate Left";

        } else if (gamepad1.circle){
            // Rotate Right
            powers = bot.calculateMotorPowers(0,0,-1);
            bot.setMotorPowers(powers[0], powers[1], powers[2], powers[3], SPEED_MULTIPLIER);
            movementDireciton = "Rotate Right";

        } else if (gamepad1.left_bumper || gamepad1.right_bumper) {
            powers = bot.calculateMotorPowers(0,0,0);
            bot.setMotorPowers(powers[0], powers[1], powers[2], powers[3], SPEED_MULTIPLIER);
            movementDireciton = "Stop";
        }

        telemetry.addData("dir:", movementDireciton);

        telemetry.addData("\nFL: ", bot.leftFrontDrive.getPower());
        telemetry.addData("BL: ", bot.leftBackDrive.getPower());
        telemetry.addData("FR: ", bot.rightFrontDrive.getPower());
        telemetry.addData("BR: ", bot.rightBackDrive.getPower());

        telemetry.addData("\n Power: ", SPEED_MULTIPLIER);

        telemetry.addData("\nFL: ", c_FL_WeightTuning);
        telemetry.addData("BL: ", c_BL_WeightTuning);
        telemetry.addData("FR: ", c_FR_WeightTuning);
        telemetry.addData("BR: ", c_BR_WeightTuning);

        telemetry.update();
    }


}
