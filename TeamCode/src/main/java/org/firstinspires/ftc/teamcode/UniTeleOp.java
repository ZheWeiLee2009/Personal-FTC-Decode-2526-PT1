package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.teamcode.Config.RobotConstants.c_DriveSpeed;
import static org.firstinspires.ftc.teamcode.Config.RobotConstants.recoveryDelay;
import static org.firstinspires.ftc.teamcode.Config.RobotConstants.recoveryPause;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Config.Drivetrain; // Robo Config

@TeleOp(name = "Universal TeleOP", group = ".")
public class UniTeleOp extends OpMode {
    Drivetrain bot;

    private ElapsedTime opmodeTimer = new ElapsedTime();
    private ElapsedTime gateTimer = new ElapsedTime();
    private double SPEED_MULTIPLIER = c_DriveSpeed;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    private boolean GateState = true;
    private boolean isWaitingGateState = false;

    private int MAX_CYCLES = 3;
    private int cycleCounter = 0;

    @Override
    public void init() {
        opmodeTimer.reset();

        bot = new Drivetrain(hardwareMap, opmodeTimer);
        bot.setServoPos(true);
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
    }


    @Override
    public void start() {
        opmodeTimer.reset();
        bot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {

        // Drive Speed
        if (gamepad1.dpad_down) {
            SPEED_MULTIPLIER = 0.5;
        }
        if (gamepad1.x) {
            SPEED_MULTIPLIER = 0.9;
        }

        // Directional Movements
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        double[] powers = bot.calculateMotorPowers(y,x,rx);
        bot.setMotorPowers(powers[0], powers[1], powers[2], powers[3], SPEED_MULTIPLIER);

        // Flywheel
        if (gamepad1.dpad_right) {
            bot.setFlywheel("full");
        } else if (gamepad1.dpad_up) {
            bot.setFlywheel("half");
        } else if (gamepad1.dpad_left) {
            bot.setFlywheel("off");
        }

        // Intake
        if (gamepad1.circle) {
            bot.setIntake("full");
        } else if (gamepad1.triangle) {
            bot.setIntake("half");
        } else if (gamepad1.square) {
            bot.setIntake("off");
        }

        // Gate single
        if (gamepad1.leftBumperWasPressed() && gateTimer.milliseconds() >= recoveryPause) {
            GateState = isWaitingGateState;
            bot.setServoPos(GateState);
            if (!isWaitingGateState) {
                gateTimer.reset();
            } else {
                cycleCounter =0;
            }
            isWaitingGateState = !isWaitingGateState;
        }

        // Gate multi
        if (gamepad1.rightBumperWasPressed() && gateTimer.milliseconds() >= recoveryPause){
            GateState = isWaitingGateState;
            bot.setServoPos(GateState);
            if (!isWaitingGateState) {
                gateTimer.reset();
                cycleCounter = 1; // Start multi-cycle
            } else {
                cycleCounter = 0; // Cancel cycles
            }
            isWaitingGateState = !isWaitingGateState;
        }

        // Check
        if (isWaitingGateState && gateTimer.milliseconds() >= recoveryPause) {
            GateState = true;
            bot.setServoPos(true);
            isWaitingGateState = false;

            if (cycleCounter > 0 && cycleCounter < MAX_CYCLES) {
                cycleCounter++;
                gateTimer.reset();
            } else {
                cycleCounter = 0;
            }
        }
        // multi cycle Continuation

        if (!isWaitingGateState && cycleCounter > 0 && gateTimer.milliseconds() >= recoveryDelay) {
            GateState = false;
            bot.setServoPos(false);
            isWaitingGateState = true;
            gateTimer.reset();
        }

        // redundancy
        if (GateState) {
            bot.setServoPos(true);
        }

        // Telemetry
        telemetry.addData("Flywheel: ", bot.Flywheel.getPower());
        telemetry.addData("Intake: ", bot.Intake.getPower());
        telemetry.addData("GatePOS: ", bot.Gate.getPosition());
        telemetry.addData("GateTimer:", gateTimer.milliseconds());
        telemetry.addData("GateState", GateState);
        telemetry.addData("GatePause:", recoveryPause);

        telemetry.addData("\nFL: ", bot.leftFrontDrive.getPower());
        telemetry.addData("BL: ", bot.leftBackDrive.getPower());
        telemetry.addData("FR: ", bot.rightFrontDrive.getPower());
        telemetry.addData("BR: ", bot.rightBackDrive.getPower());

        telemetry.addData("\n\n Full Power: ", SPEED_MULTIPLIER);

        telemetry.update();



        // FTC Dashboard Telemetry + Graph
        dashboardTelemetry.addData("Flywheel Power: ", bot.Flywheel.getPower());
        dashboardTelemetry.addData("Intake Power: ", bot.Intake.getPower());
        dashboardTelemetry.addData("FL: ", bot.leftFrontDrive.getPower());
        dashboardTelemetry.addData("BL: ", bot.leftBackDrive.getPower());
        dashboardTelemetry.addData("FR: ", bot.rightFrontDrive.getPower());
        dashboardTelemetry.addData("BR: ", bot.rightBackDrive.getPower());

        dashboardTelemetry.addData("\nGateTimer:", gateTimer.milliseconds());
        dashboardTelemetry.addData("GateState", GateState);
        dashboardTelemetry.addData("GatePause:", recoveryPause);

        telemetry.addData("\n\n Power: ", SPEED_MULTIPLIER);

        dashboardTelemetry.update();

    }


}
