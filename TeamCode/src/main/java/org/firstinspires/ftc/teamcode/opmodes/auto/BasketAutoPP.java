package org.firstinspires.ftc.teamcode.opmodes.auto;
import androidx.collection.ArraySet;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.commands.ActionCommand;
import org.firstinspires.ftc.teamcode.commands.AscentOpenHooksCommand;
import org.firstinspires.ftc.teamcode.commands.CloseGripplerCommand;
import org.firstinspires.ftc.teamcode.commands.ColourAwareIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeOffCommand;
import org.firstinspires.ftc.teamcode.commands.IntakePivotDownCommand;
import org.firstinspires.ftc.teamcode.commands.IntakePivotUpCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeSlidesInCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeSlidesOutCommand;
import org.firstinspires.ftc.teamcode.commands.OpenGripplerCommand;
import org.firstinspires.ftc.teamcode.commands.SlidesStowCommand;
import org.firstinspires.ftc.teamcode.commands.TransferFlipCommand;
import org.firstinspires.ftc.teamcode.commands.TransferStowCommand;
import org.firstinspires.ftc.teamcode.commands.groups.DeliveryCommandGroup;
import org.firstinspires.ftc.teamcode.commands.groups.DeliveryResetCommandGroup;
import org.firstinspires.ftc.teamcode.commands.groups.IntakeCommandGroup;
import org.firstinspires.ftc.teamcode.subsystems.AscentSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotStateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlidesSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;
import org.firstinspires.ftc.teamcode.utils.OTOSDrive;
import org.firstinspires.ftc.teamcode.utils.PinpointDrive;
import org.firstinspires.ftc.teamcode.utils.PoseStorage;

@Autonomous(name = "SPEC | THREE | PARK", group = "Autonomous")

public class BasketAutoPP extends CommandOpMode {

    TrajectoryActionBuilder dropOffPreload;
    TrajectoryActionBuilder firstSample;

    TrajectoryActionBuilder firstSampleSlowMoveIn;
    TrajectoryActionBuilder deliverFirstSample;
    TrajectoryActionBuilder deliverFirstSampleMoveIn;
    TrajectoryActionBuilder secondSample;
    TrajectoryActionBuilder secondSampleSlowMoveIn;
    TrajectoryActionBuilder deliverSecondSample;
    TrajectoryActionBuilder deliverSecondSampleMoveIn;
    TrajectoryActionBuilder thirdSample;
    TrajectoryActionBuilder thirdSampleSlowMoveIn;
    TrajectoryActionBuilder deliverThirdSample;

    TrajectoryActionBuilder deliverThirdSampleMoveIn;
    TrajectoryActionBuilder park;


    Action pleaseWork;
    IntakeSubsystem intakeSubsystem;

    TransferSubsystem transferSubsystem;

    RobotStateSubsystem robotState;

    SlidesSubsystem slidesSubsystem;

    AscentSubsystem ascentSubsystem;

    //Change these offsets, they can be negative values
    int X_OFFSET = 0; // a larger negative number takes it closer to the basket
    int Y_OFFSET = 0; // a larger number takes it closer to the submersible

    @Override
    public void initialize() {

        intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetry);
        transferSubsystem = new TransferSubsystem(hardwareMap);
        robotState = new RobotStateSubsystem();
        slidesSubsystem = new SlidesSubsystem(hardwareMap, telemetry);
        ascentSubsystem = new AscentSubsystem(hardwareMap);


        // instantiate your MecanumDrive at a particular pose.
        PinpointDrive drive = new PinpointDrive(hardwareMap,
                new Pose2d(-14.8, -64, Math.toRadians(-90)));

        //pose to the submersible wall
        Pose2d dropOffPose = new Pose2d(-3.4, -32.5, Math.toRadians(-90));

        dropOffPreload = drive.actionBuilder(drive.pose)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(dropOffPose, Math.toRadians(90))
                .endTrajectory();


        firstSample = dropOffPreload.fresh()
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-3.4, -36, Math.toRadians(-90)), Math.toRadians(-90))
                .setTangent(Math.toRadians(-131))
                .splineToLinearHeading(new Pose2d(-37,-65,Math.toRadians(-235)),Math.toRadians(-131))
                .endTrajectory();


        //slow move in
        Pose2d slowMoveForward = new Pose2d(-37 + X_OFFSET, -52 + Y_OFFSET, Math.toRadians(-235));

        firstSampleSlowMoveIn = firstSample.fresh()
                .setTangent(Math.toRadians(-260))
                .splineToLinearHeading(slowMoveForward,Math.toRadians(-260), new TranslationalVelConstraint(5))
                .endTrajectory();

        Pose2d deliverPose = new Pose2d(-47 + X_OFFSET,-54 + Y_OFFSET,Math.toRadians(-315));

        deliverFirstSample = firstSampleSlowMoveIn.fresh()
                //pick up the first sample
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(deliverPose,Math.toRadians(180))
                .endTrajectory();

        Pose2d deliverFirstSampleMoveInPose = new Pose2d(-60 + X_OFFSET,-56 + Y_OFFSET, Math.toRadians(-315));

        deliverFirstSampleMoveIn = deliverFirstSample.fresh()
                .splineToLinearHeading(deliverFirstSampleMoveInPose, Math.toRadians(-90))
                .endTrajectory();



        Pose2d secondSamplePose = new Pose2d(-58 + X_OFFSET,-65 + Y_OFFSET,Math.toRadians(100));

        secondSample = deliverFirstSampleMoveIn.fresh()
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(secondSamplePose,Math.toRadians(90))
                .endTrajectory();

        Pose2d secondSampleMoveInPose = new Pose2d(-58 + X_OFFSET,-56 + Y_OFFSET,Math.toRadians(100));


        secondSampleSlowMoveIn = secondSample.fresh()
                .splineToLinearHeading(secondSampleMoveInPose, Math.toRadians(90), new TranslationalVelConstraint(5))
                .endTrajectory();

        Pose2d deliverSecondSamplePose = new Pose2d(-56 + X_OFFSET,-55 + Y_OFFSET,Math.toRadians(-315));

        deliverSecondSample = secondSampleSlowMoveIn.fresh()
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(deliverSecondSamplePose,Math.toRadians(-90))
                .endTrajectory();

        Pose2d deliverSecondSampleMoveInPose = new Pose2d(-62 + X_OFFSET,-58 + Y_OFFSET, Math.toRadians(-315));


        deliverSecondSampleMoveIn = deliverSecondSample.fresh()
                .splineToLinearHeading(deliverSecondSampleMoveInPose, Math.toRadians(-90))
                .endTrajectory();

        Pose2d thirdSamplePose = new Pose2d(-60 + X_OFFSET,-65 + Y_OFFSET,Math.toRadians(-242));

        thirdSample = deliverSecondSampleMoveIn.fresh()
                .setTangent(Math.toRadians(103))
                .splineToLinearHeading(thirdSamplePose,Math.toRadians(103))
                .endTrajectory();

        Pose2d thirdSampleMoveInPose = new Pose2d(-62 + X_OFFSET,-55 + Y_OFFSET,Math.toRadians(-242));

        thirdSampleSlowMoveIn = thirdSample.fresh()
                .splineToLinearHeading(thirdSampleMoveInPose, Math.toRadians(90), new TranslationalVelConstraint(6))
                .endTrajectory();

        Pose2d deliverThirdMovePose = new Pose2d(-60 + X_OFFSET,-53 + Y_OFFSET,Math.toRadians(-315));

        deliverThirdSample = thirdSampleSlowMoveIn.fresh()
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(deliverThirdMovePose , Math.toRadians(-315))
                .endTrajectory();

        Pose2d deliverThirdSampleMoveInPose = new Pose2d(-60 + X_OFFSET,-58 + Y_OFFSET, Math.toRadians(-315));

        deliverThirdSampleMoveIn = deliverThirdSample.fresh()
                .splineToLinearHeading(deliverThirdSampleMoveInPose, Math.toRadians(-90))
                .endTrajectory();

        park = deliverThirdSampleMoveIn.fresh()
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-10,-10,Math.toRadians(180)),Math.toRadians(0))
                .endTrajectory();

        intakeSubsystem.setDesiredColour(IntakeSubsystem.SampleColour.NEUTRAL);
        intakeSubsystem.intakePivotDown();
        transferSubsystem.closeGrippler();


        CommandScheduler.getInstance().schedule(
                new WaitUntilCommand(this::isStarted).andThen(
                    new SequentialCommandGroup(

                            new ActionCommand(dropOffPreload.build(), new ArraySet<>()),

                            new WaitCommand(100),
                            new TransferFlipCommand(transferSubsystem),
                            new WaitCommand(500),
                            new SequentialCommandGroup(

                                    new OpenGripplerCommand(transferSubsystem),
                                    new TransferStowCommand(transferSubsystem),

                                    new IntakePivotUpCommand(intakeSubsystem, robotState),
                                    new SlidesStowCommand(slidesSubsystem),
                                    new InstantCommand(()->{
                                        slidesSubsystem.NoPowerSlides();
                                    })
                            ),
                            // specimen is now delivered

                            new ActionCommand(firstSample.build(), new ArraySet<>()),

                            new SequentialCommandGroup(
                                    new IntakeSlidesOutCommand(intakeSubsystem),
                                    new IntakePivotDownCommand(intakeSubsystem, robotState)//,
                             ),
                            new ParallelCommandGroup(
                                    new SequentialCommandGroup(
                                            new WaitCommand(300), //give time for the movement to start
                                            new ColourAwareIntakeCommand(intakeSubsystem).withTimeout(2000)
                                    ),
                                    new ActionCommand(firstSampleSlowMoveIn.build(), new ArraySet<>())
                            ),
                            new IntakeCommandGroup(intakeSubsystem, transferSubsystem, robotState),
                            new CloseGripplerCommand(transferSubsystem),
                            new WaitCommand(200),

                             new SequentialCommandGroup(
                                    new ParallelCommandGroup(

                                            new ActionCommand(deliverFirstSample.build(), new ArraySet<>()),
                                            new DeliveryCommandGroup(intakeSubsystem, transferSubsystem, slidesSubsystem, robotState )

                                    ),
                                     new ActionCommand(deliverFirstSampleMoveIn.build(), new ArraySet<>()),
                                     new OpenGripplerCommand(transferSubsystem),
                                     new WaitCommand(250)

                             ),

                            new ParallelCommandGroup(

                                    new ActionCommand(secondSample.build(), new ArraySet<>()),
                                    new DeliveryResetCommandGroup(intakeSubsystem,transferSubsystem,slidesSubsystem, robotState)
                            ),
                            new SequentialCommandGroup(
                                    new SequentialCommandGroup(
                                            new IntakeSlidesOutCommand(intakeSubsystem),
                                            new IntakePivotDownCommand(intakeSubsystem, robotState)
                                            ),
                                    new ParallelCommandGroup(
                                        new SequentialCommandGroup(
                                            new WaitCommand(300),
                                            new ColourAwareIntakeCommand(intakeSubsystem).withTimeout(2000)
                                        ),
                                        new ActionCommand(secondSampleSlowMoveIn.build(), new ArraySet<>())
                                    )

                            ),

                            new IntakeCommandGroup(intakeSubsystem, transferSubsystem, robotState),


                            new SequentialCommandGroup(
                                    new CloseGripplerCommand(transferSubsystem),

                                            // do the drop off if we have the sample
                                            new SequentialCommandGroup(
                                                    new ParallelCommandGroup(
                                                            new ActionCommand(deliverSecondSample.build(), new ArraySet<>()),
                                                            new DeliveryCommandGroup(intakeSubsystem, transferSubsystem, slidesSubsystem, robotState )
                                                    ),
                                                    new ActionCommand(deliverSecondSampleMoveIn.build(), new ArraySet<>()),
                                                    new OpenGripplerCommand(transferSubsystem),
                                                    new WaitCommand(250)
                                            )
                            ),


                            new ParallelCommandGroup(

                                    new ActionCommand(thirdSample.build(), new ArraySet<>()),
                                    new DeliveryResetCommandGroup(intakeSubsystem,transferSubsystem,slidesSubsystem, robotState)
                            ),
                            //third sample


                            new ParallelCommandGroup(
                                    new SequentialCommandGroup(
                                            new IntakeSlidesOutCommand(intakeSubsystem),
                                            new IntakePivotDownCommand(intakeSubsystem, robotState),
                                            new ColourAwareIntakeCommand(intakeSubsystem).withTimeout(2000)
                                    ),
                                    new ActionCommand(thirdSampleSlowMoveIn.build(), new ArraySet<>())
                            ),


                            new IntakeCommandGroup(intakeSubsystem, transferSubsystem, robotState),


                            new SequentialCommandGroup(
                                    new ParallelCommandGroup(
                                            new ActionCommand(deliverThirdSample.build(), new ArraySet<>()),
                                            new DeliveryCommandGroup(intakeSubsystem, transferSubsystem, slidesSubsystem, robotState )
                                    ),
                                    new ActionCommand(deliverThirdSampleMoveIn.build(), new ArraySet<>()),
                                    new OpenGripplerCommand(transferSubsystem),
                                    new WaitCommand(250)
                            ),


                            new ParallelCommandGroup(

                                    new ActionCommand(park.build(), new ArraySet<>()),
                                    new DeliveryResetCommandGroup(intakeSubsystem,transferSubsystem,slidesSubsystem, robotState),
                                    new AscentOpenHooksCommand(ascentSubsystem),
                                    new InstantCommand(()->{

                                        PoseStorage.currentPose = new Pose2d(0, 0, Math.toRadians(-270));

                                    })

                            )
                    )
                )
        );

    }

}
