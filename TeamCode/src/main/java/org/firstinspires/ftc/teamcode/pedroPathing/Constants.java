package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    /*
    Translational PIDF:

    P = 0.06
    I = ??
    D = ??
    F = ??
    */

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(8)
            .forwardZeroPowerAcceleration(-21.16128)
            .lateralZeroPowerAcceleration(-44.73058)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.06, 0, 0.0, 0.01))
            .headingPIDFCoefficients(new PIDFCoefficients(1.2, 0, 0.01, 0.01))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.018,0.0,0.00002,0.5,0.01))
            .centripetalScaling(0.005);



    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("FR")
            .rightRearMotorName("BR")
            .leftRearMotorName("BL")
            .leftFrontMotorName("FL")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(45.42)
            .yVelocity(38.45);


    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(6)
            .strafePodX(2)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("POC")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
