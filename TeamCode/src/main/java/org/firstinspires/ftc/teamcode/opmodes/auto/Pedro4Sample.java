package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.commands.SlidesHighBasketCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.subsystems.SlidesSubsystem;

@Autonomous
public class Pedro4Sample extends CommandOpMode {
    Follower f;
    SlidesSubsystem slides;

    public static PathBuilder builder = new PathBuilder();

    public static PathChain line1 = builder
            .addPath(
                    new BezierLine(
                            new Point(7.550, 112.450, Point.CARTESIAN),
                            new Point(16.000, 128.000, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
            .build();

    public static PathChain line2 = builder
            .addPath(
                    new BezierLine(
                            new Point(16.000, 128.000, Point.CARTESIAN),
                            new Point(32.500, 121.000, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(0))
            .build();

    public static PathChain line3 = builder
            .addPath(
                    new BezierLine(
                            new Point(32.500, 121.000, Point.CARTESIAN),
                            new Point(16.000, 128.000, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
            .build();

    public static PathChain line4 = builder
            .addPath(
                    new BezierLine(
                            new Point(16.000, 128.000, Point.CARTESIAN),
                            new Point(32.500, 132.000, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(0))
            .build();

    public static PathChain line5 = builder
            .addPath(
                    new BezierLine(
                            new Point(32.500, 132.000, Point.CARTESIAN),
                            new Point(16.000, 128.000, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
            .build();

    public static PathChain line6 = builder
            .addPath(
                    new BezierLine(
                            new Point(16.000, 128.000, Point.CARTESIAN),
                            new Point(32.500, 132.000, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(45))
            .build();

    public static PathChain line7 = builder
            .addPath(
                    new BezierLine(
                            new Point(32.500, 132.000, Point.CARTESIAN),
                            new Point(16.000, 128.000, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(-45))
            .build();

    @Override
    public void initialize() {
        f = new Follower(hardwareMap, FConstants.class, LConstants.class);
        f.setStartingPose(new Pose(7.55, 112.450, 0));

        slides = new SlidesSubsystem(hardwareMap, telemetry);
        register(slides);


        schedule(
                new FollowPathCommand(f, line1, false)
                        .alongWith(
                                new SlidesHighBasketCommand(slides)
                        ),
                new FollowPathCommand(f, line2, false),
                new FollowPathCommand(f, line3, false),
                new FollowPathCommand(f, line4, false),
                new FollowPathCommand(f, line5, false),
                new FollowPathCommand(f, line6, false),
                new FollowPathCommand(f, line7, false)
        );
    }
}
