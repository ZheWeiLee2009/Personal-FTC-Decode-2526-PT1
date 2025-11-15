package org.firstinspires.ftc.teamcode.Config;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class EthanPaths {
    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path4;
    public PathChain Path5;

    // heading that aims the robot at the bucket (tune this angle on-field)
    private static final double BUCKET_HEADING = Math.toRadians(131);

    public EthanPaths(Follower follower) {
        Path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(23.931, 129.765),
                                new Pose(48.481, 96.138),
                                new Pose(58.178, 84.172)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(143), BUCKET_HEADING)
                .build();

        Path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(58.178, 84.172),
                                new Pose(43.530, 83.966),
                                new Pose(27.089, 83.762)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        // ⬇️ Return from first stack: always end facing BUCKET_HEADING
        Path3 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(27.089, 83.762),
                                new Pose(71.827, 75.462),
                                new Pose(58.590, 83.966)
                        )
                )
                // keep driving the same curve, but hold heading toward the bucket
                .setConstantHeadingInterpolation(BUCKET_HEADING)
                .build();

        Path4 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(58.590, 83.966),
                                new Pose(84.808, 63.000),
                                new Pose(27.446, 59.525)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        // ⬇️ Return from second stack: also end facing BUCKET_HEADING
        Path5 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(27.446, 59.525),
                                new Pose(88.298, 63.542),
                                new Pose(58.590, 84.172)
                        )
                )
                .setConstantHeadingInterpolation(BUCKET_HEADING)
                .build();
    }
}
