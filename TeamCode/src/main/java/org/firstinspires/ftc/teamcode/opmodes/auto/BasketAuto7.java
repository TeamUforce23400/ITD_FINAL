// File: teamcode/opmodes/auto/BasketAuto7.java
package org.firstinspires.ftc.teamcode.opmodes.auto;

import androidx.collection.ArraySet;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.CloseGripplerCommand;
import org.firstinspires.ftc.teamcode.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.commands.SlidesHighBasketCommand;
import org.firstinspires.ftc.teamcode.commands.TransferFlipCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotStateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlidesSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;

@Autonomous(name = "Test 5 Spec", group = "Autonomous")
public class BasketAuto7 extends CommandOpMode {

    Follower f;
    SlidesSubsystem slides;

    IntakeSubsystem     intakeSubsystem;
    TransferSubsystem   transferSubsystem;
    RobotStateSubsystem robotState;

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
        // SET UP PEDRO PATHING
        Constants.setConstants(FConstants.class, LConstants.class);
        f = new Follower(hardwareMap, FConstants.class, LConstants.class);
        f.setStartingPose(new Pose(7.55, 112.450, 0));

        slides = new SlidesSubsystem(hardwareMap, telemetry);
        intakeSubsystem   = new IntakeSubsystem(hardwareMap, telemetry);
        transferSubsystem = new TransferSubsystem(hardwareMap);
        robotState = new RobotStateSubsystem();

        transferSubsystem.closeGrippler();

        // BUILD PATHS


        // SCHEDULE AUTO
        schedule(
                        new SequentialCommandGroup(
                                // reset pose on the road-runner pose tracker
                                new FollowPathCommand(f, line1, false)
                                        .alongWith(
                                                new SlidesHighBasketCommand(slides),
                                                new TransferFlipCommand(transferSubsystem)
                                        )
//                                new FollowPathCommand(follower, path2, false)
                        )
                );
    }

//    @Override
//    public void run() {
//        // run the scheduled commands and update follower
//        super.run();
//        f.update();
//    }
}
