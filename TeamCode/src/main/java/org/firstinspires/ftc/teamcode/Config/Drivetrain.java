package org.firstinspires.ftc.teamcode.Config;

import static org.firstinspires.ftc.teamcode.Config.RobotConstants.Gate_Off;
import static org.firstinspires.ftc.teamcode.Config.RobotConstants.Gate_On;
import static org.firstinspires.ftc.teamcode.Config.RobotConstants.INTAKE_OUT;
import static org.firstinspires.ftc.teamcode.Config.RobotConstants.c_BL_WeightTuning;
import static org.firstinspires.ftc.teamcode.Config.RobotConstants.c_BR_WeightTuning;
import static org.firstinspires.ftc.teamcode.Config.RobotConstants.c_FL_WeightTuning;
import static org.firstinspires.ftc.teamcode.Config.RobotConstants.c_FR_WeightTuning;
import static org.firstinspires.ftc.teamcode.Config.RobotConstants.FLYWHEEL_Full;
import static org.firstinspires.ftc.teamcode.Config.RobotConstants.FLYWHEEL_Half;
import static org.firstinspires.ftc.teamcode.Config.RobotConstants.FLYWHEEL_OFF;
import static org.firstinspires.ftc.teamcode.Config.RobotConstants.INTAKE_Full;
import static org.firstinspires.ftc.teamcode.Config.RobotConstants.INTAKE_Half;
import static org.firstinspires.ftc.teamcode.Config.RobotConstants.INTAKE_OFF;

import static org.firstinspires.ftc.teamcode.Config.FieldOrientedConstants.iOffsetX;
import static org.firstinspires.ftc.teamcode.Config.FieldOrientedConstants.iOffsetY;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Arrays;
import java.util.List;

public class Drivetrain {

    HardwareMap hwMap;


    public double oldTime=0;


    // Drive
    public DcMotorEx leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive;
    private final  List<DcMotorEx> motors;


    // Aux
    public DcMotorEx Flywheel, Intake;

    public Servo Gate;
    // States
    public String flywheelState, intakeState;

    public Drivetrain(HardwareMap hwMapX, ElapsedTime runtime) {
        hwMap = hwMapX;

        // Drive
        leftFrontDrive = hwMap.get(DcMotorEx.class,"FL");
        leftBackDrive = hwMap.get(DcMotorEx.class,"BL");
        rightFrontDrive = hwMap.get(DcMotorEx.class,"FR");
        rightBackDrive = hwMap.get(DcMotorEx.class,"BR");

        motors = Arrays.asList(leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive);

        // Aux
        Intake = hwMap.get(DcMotorEx.class,"Intake");
        Flywheel = hwMap.get(DcMotorEx.class,"Flywheel");

        Gate = hwMap.get(Servo.class, "Gate");



        // Set Modes:
        setMotorsMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);

    }

    public double[] calculateMotorPowers(double axial, double lateral, double yaw) {
        double[] motorPowers = new double[4];
        motorPowers[0] = (axial + lateral + yaw);
        motorPowers[1] = (axial - lateral + yaw);
        motorPowers[2] = (axial - lateral - yaw);
        motorPowers[3] = (axial + lateral - yaw);
        return motorPowers;
    }

    public void setMotorPowers(double v, double v1, double v2, double v3, double speedMultiplier) {
        leftFrontDrive.setPower(v * speedMultiplier * c_FL_WeightTuning);
        leftBackDrive.setPower(v1 * speedMultiplier * c_BL_WeightTuning);
        rightFrontDrive.setPower(v2 * speedMultiplier * c_FR_WeightTuning);
        rightBackDrive.setPower(v3 * speedMultiplier * c_BR_WeightTuning);
    }

    public void setMotorsMode(DcMotorEx.RunMode mode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(mode);
        }
    }
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zpb) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zpb);
        }
    }

    public void setServoPos(Boolean dir) {
        if (!dir) {
            Gate.setPosition(Gate_On);
        } else {// redundancy
            Gate.setPosition(Gate_Off);
        }
    }

    public void setFlywheel(String dir, double offset) {
        switch (dir) {
            case "full":
                Flywheel.setPower(FLYWHEEL_Full + offset);
                flywheelState = "full";
                break;
            case "half":
                Flywheel.setPower(FLYWHEEL_Half + offset);
                flywheelState = "half";
                break;
            case "less-half":
                Flywheel.setPower(FLYWHEEL_Half + offset - 0.1);
                flywheelState = "less-half";
                break;
            case "off":
                Flywheel.setPower(FLYWHEEL_OFF + offset);
                flywheelState = "off";
                break;
        }
    }

    public String getFlywheelState() {
        return flywheelState;
    }

    public void setIntake(String dir) {
        switch (dir) {
            case "full":
                Intake.setPower(INTAKE_Full);
                intakeState = "full";
                break;
            case "half":
                Intake.setPower(INTAKE_Half);
                intakeState = "half";
                break;
            case "out":
                Intake.setPower(INTAKE_OUT);
                intakeState = "out";
                break;
            case "off":
                Intake.setPower(INTAKE_OFF);
                intakeState = "off";
                break;
        }
    }

    public String getIntakeState() {
        return intakeState;
    }

//    // ****** Field Oriented Calculations ******* //
//
//    // Point Oriented
//    public double calculateHeading(){
//        Pose2D pos = odo.getPosition();
//
//        double deltaX = offsetX - robotX;
//        double deltaY = offsetY - robotY;
//
//        double calculateHeading = 0;
//
//        return calculateHeading;
//    }
//    // ~ Oriented Calculations
//    public double[] calculateFODMotorPowers(double axial, double lateral, double yaw) {
//
//        Pose2D pos = odo.getPosition();
//
//        double[] motorPowers = new double[4];
//        // rotation matrix
//        double rotX = lateral * Math.cos(-pos.getHeading(AngleUnit.RADIANS)) - axial * Math.sin(-pos.getHeading(AngleUnit.RADIANS));
//        double rotY = lateral * Math.sin(-pos.getHeading(AngleUnit.RADIANS)) + axial * Math.cos(-pos.getHeading(AngleUnit.RADIANS));
//
//        motorPowers[0] = (rotY + rotX + yaw);
//        motorPowers[1] = (rotY - rotX + yaw);
//        motorPowers[2] = (rotY - rotX - yaw);
//        motorPowers[3] = (rotY + rotX - yaw);
//
//        return motorPowers;
//
//    }


}
