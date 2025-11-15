package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Config.Drivetrain; // Robo Config
import org.firstinspires.ftc.teamcode.Config.EthanPaths;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "EthanEXPedroMulti", group = "Autonomous")
public class EthanPedroExAuto extends LinearOpMode {

    private Drivetrain bot;
    private Follower follower;
    private EthanPaths paths;

    @Override
    public void runOpMode() throws InterruptedException {

        // --- Drivetrain / mechanisms ---
        bot = new Drivetrain(hardwareMap, new ElapsedTime());
        bot.setServoPos(true);          // gate CLOSED at start
        bot.setIntake("off");
        bot.setFlywheel("off", 0);

        // --- Pedro follower + paths ---
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(24, 129.77, Math.toRadians(143)));
        paths = new EthanPaths(follower);

        telemetry.addLine("Ethan Pedro Pathing Ready!");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // =============================
        //         AUTON SEQUENCE
        // =============================

        // ---------- PRELOADS ----------
        // Intake ON immediately and stay on until after 3 preloads are outtaken.
        bot.setIntake("full");
        follow(paths.Path1);           // drive to the line with intake holding balls

        // Flywheel + gate open to shoot 3 preloads
        bot.setFlywheel("full", 0);
        bot.setServoPos(false);        // OPEN gate so balls can feed
        sleep(2500);                   // 2.5s shoot time
        bot.setServoPos(true);         // CLOSE gate
        bot.setFlywheel("off", 0);
        bot.setIntake("off");          // done with preload set

        // ---------- CYCLE 1 ----------
        // Collect 3, keep intake on while holding and shooting them.
        bot.setIntake("full");
        follow(paths.Path2);           // drive through first stack (collect)
        follow(paths.Path3);           // drive back to line still holding with intake

        // Shoot those 3: flywheel + gate
        bot.setFlywheel("full", 0);
        bot.setServoPos(false);        // OPEN gate
        sleep(2500);
        bot.setServoPos(true);         // CLOSE gate
        bot.setFlywheel("off", 0);
        bot.setIntake("off");          // done with this set of 3

        // ---------- CYCLE 2 ----------
        // Same pattern for the second set of 3.
        bot.setIntake("full");
        follow(paths.Path4);           // drive through second stack (collect)
        follow(paths.Path5);           // drive back to line with intake still on

        bot.setFlywheel("full", 0);
        bot.setServoPos(false);        // OPEN gate
        sleep(2500);
        bot.setServoPos(true);         // CLOSE gate
        bot.setFlywheel("off", 0);
        bot.setIntake("off");          // done holding balls

        // ---------- END ----------
        telemetry.addLine("✅ All paths complete, 9 balls outtaken.");
        telemetry.update();
        sleep(1000);
    }

    /** Helper method to follow one path and wait for completion */
    private void follow(com.pedropathing.paths.PathChain path) {
        follower.followPath(path);

        while (opModeIsActive() && follower.isBusy()) {
            follower.update();

            telemetry.addData("Following", path);
            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.update();
        }

        follower.breakFollowing();
        sleep(250); // small pause between paths (optional)
    }
}