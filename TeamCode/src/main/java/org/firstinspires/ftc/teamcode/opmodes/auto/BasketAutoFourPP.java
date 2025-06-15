//package org.firstinspires.ftc.teamcode.opmodes.auto;
//import androidx.collection.ArraySet;
//
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
//import com.acmerobotics.roadrunner.TranslationalVelConstraint;
//import com.arcrobotics.ftclib.command.CommandOpMode;
//import com.arcrobotics.ftclib.command.CommandScheduler;
//import com.arcrobotics.ftclib.command.InstantCommand;
//import com.arcrobotics.ftclib.command.ParallelCommandGroup;
//import com.arcrobotics.ftclib.command.SequentialCommandGroup;
//import com.arcrobotics.ftclib.command.WaitCommand;
//import com.arcrobotics.ftclib.command.WaitUntilCommand;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
//import org.firstinspires.ftc.teamcode.roadrunner.tuning.MecanumDrive;
//import org.firstinspires.ftc.teamcode.commands.ActionCommand;
//import org.firstinspires.ftc.teamcode.commands.CloseGripplerCommand;
//import org.firstinspires.ftc.teamcode.commands.ColourAwareIntakeCommand;
//import org.firstinspires.ftc.teamcode.commands.IntakePivotDownCommand;
//import org.firstinspires.ftc.teamcode.commands.IntakeSlidesOutCommand;
//import org.firstinspires.ftc.teamcode.commands.OpenGripplerCommand;
//import org.firstinspires.ftc.teamcode.commands.TransferFlipCommand;
//import org.firstinspires.ftc.teamcode.commands.groups.DeliveryCommandGroup;
//import org.firstinspires.ftc.teamcode.commands.groups.DeliveryResetCommandGroup;
//import org.firstinspires.ftc.teamcode.commands.groups.IntakeCommandGroup;
//import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.RobotStateSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.SlidesSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;
//
//@Autonomous(name = "BASKET | FOUR | PARK", group = "Autonomous")
//public class BasketAutoFourPP extends CommandOpMode {
//
//    TrajectoryActionBuilder dropOffPreload;
//    TrajectoryActionBuilder apSample; //ap = alliance preload
//
//    TrajectoryActionBuilder apSlowMoveIn;
//    TrajectoryActionBuilder deliverAPSample;
//    TrajectoryActionBuilder deliverAPSampleMoveIn;
//
//    TrajectoryActionBuilder firstSample;
//
//    TrajectoryActionBuilder firstSampleSlowMoveIn;
//
//    TrajectoryActionBuilder firstSampleDeliver;
//
//    TrajectoryActionBuilder firstSampleDeliverIn;
//    TrajectoryActionBuilder secondSample;
//    TrajectoryActionBuilder secondSampleSlowMoveIn;
//    TrajectoryActionBuilder deliverSecondSample;
//    TrajectoryActionBuilder deliverSecondSampleMoveIn;
//    TrajectoryActionBuilder thirdSample;
//    TrajectoryActionBuilder thirdSampleSlowMoveIn;
//    TrajectoryActionBuilder deliverThirdSample;
//
//    TrajectoryActionBuilder deliverThirdSampleMoveIn;
//    TrajectoryActionBuilder pick4thMoveIn;
//    TrajectoryActionBuilder pick4thSlowMoveIn;
//    TrajectoryActionBuilder dropOff4th;
//    TrajectoryActionBuilder park;
//
//
//    Action pleaseWork;
//    IntakeSubsystem intakeSubsystem;
//
//    TransferSubsystem transferSubsystem;
//
//    RobotStateSubsystem robotState;
//
//    SlidesSubsystem slidesSubsystem;
//
//
//
//    //Change these offsets, they can be negative values
//    int X_OFFSET = 0; // a larger negative number takes it closer to the basket
//    int Y_OFFSET = 0; // a larger number takes it closer to the submersible
//
//    @Override
//    public void initialize() {
//
//        intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetry);
//        transferSubsystem = new TransferSubsystem(hardwareMap);
//        robotState = new RobotStateSubsystem();
//        slidesSubsystem = new SlidesSubsystem(hardwareMap, telemetry);
//
//
//
//        // instantiate your MecanumDrive at a particular pose.
//        MecanumDrive drive = new MecanumDrive(hardwareMap,
//                new Pose2d(-48, -62, Math.toRadians(0)));
//
//        //pose to the submersible wall
//        Pose2d dropOffPose = new Pose2d(-60.5, -62.5, Math.toRadians(0));
//
//        dropOffPreload = drive.actionBuilder(drive.pose)
//                .setTangent(Math.toRadians(0))
////                .strafeTo(new Vector2d(-48, -58))
//                .splineToLinearHeading(dropOffPose, Math.toRadians(0), new TranslationalVelConstraint(10))
//                .endTrajectory();
//
//
//
//
//
//
//
//        Pose2d firstSamplePose = new Pose2d(-53.8, -58.5, Math.toRadians(90));
//
//
//        firstSample = drive.actionBuilder(drive.pose)
//                .splineToLinearHeading(firstSamplePose, Math.toRadians(90), new TranslationalVelConstraint(11))
//                .endTrajectory();
//
//        Pose2d firstSampleSlowMoveInPose = new Pose2d(-53.8, -43.9, Math.toRadians(90));
//
//        firstSampleSlowMoveIn = firstSample.fresh()
//                .splineToLinearHeading(firstSampleSlowMoveInPose, Math.toRadians(100), new TranslationalVelConstraint(7))
//                .endTrajectory();
//
//
//        Pose2d firstSampleDeliverPose = new Pose2d(-61 + X_OFFSET,-53.3 + Y_OFFSET,Math.toRadians(-315));
//
//
//        firstSampleDeliver = firstSampleSlowMoveIn.fresh()
//                .splineToLinearHeading(firstSampleDeliverPose, Math.toRadians(90))
//                .endTrajectory();
//
//        Pose2d deliverFirstSampleMoveInPose = new Pose2d(-63 + X_OFFSET,-56 + Y_OFFSET, Math.toRadians(-315));
//
//        firstSampleDeliverIn = firstSampleDeliver.fresh()
//                .splineToLinearHeading(deliverFirstSampleMoveInPose, Math.toRadians(90))
//                .endTrajectory();
//
//        Pose2d secondSamplePose = new Pose2d(-66.2 + X_OFFSET,-59.5 + Y_OFFSET,Math.toRadians(90));
//
//
//        secondSample = firstSampleDeliverIn.fresh()
//                .setTangent(Math.toRadians(90))
//                .splineToLinearHeading(secondSamplePose,Math.toRadians(90))
//                .endTrajectory();
//
//        Pose2d secondSampleMoveInPose = new Pose2d(-66.7 + X_OFFSET,-43.5 + Y_OFFSET,Math.toRadians(90));
//
//
//        secondSampleSlowMoveIn = secondSample.fresh()
//                .splineToLinearHeading(secondSampleMoveInPose, Math.toRadians(90), new TranslationalVelConstraint(8))
//                .endTrajectory();
//
//        Pose2d deliverSecondSamplePose = new Pose2d(-60 + X_OFFSET,-55 + Y_OFFSET,Math.toRadians(-315));
//
//        deliverSecondSample = secondSampleSlowMoveIn.fresh()
//                .setTangent(Math.toRadians(90))
//                .splineToLinearHeading(deliverSecondSamplePose,Math.toRadians(90), new TranslationalVelConstraint(11))
//                .endTrajectory();
//
//        Pose2d deliverSecondSampleMoveInPose = new Pose2d(-64 + X_OFFSET,-55 + Y_OFFSET, Math.toRadians(-315));
//
//
//        deliverSecondSampleMoveIn = deliverSecondSample.fresh()
//                .splineToLinearHeading(deliverSecondSampleMoveInPose, Math.toRadians(90))
//                .endTrajectory();
//
//        Pose2d thirdSamplePose = new Pose2d(-58 + X_OFFSET,-50 + Y_OFFSET,Math.toRadians(-235));
//
//        thirdSample = deliverSecondSampleMoveIn.fresh()
//                .setTangent(Math.toRadians(140))
//                .splineToLinearHeading(thirdSamplePose,Math.toRadians(140), new TranslationalVelConstraint(9))
//                .endTrajectory();
//
//        Pose2d thirdSampleMoveInPose = new Pose2d(-62.6  + X_OFFSET,-43.0 + Y_OFFSET,Math.toRadians(-235));
//
//        thirdSampleSlowMoveIn = thirdSample.fresh()
//                .splineToLinearHeading(thirdSampleMoveInPose, Math.toRadians(90), new TranslationalVelConstraint(8))
//                .endTrajectory();
//
//        Pose2d deliverThirdMovePose = new Pose2d(-60 + X_OFFSET,-53 + Y_OFFSET,Math.toRadians(-315));
//
//        deliverThirdSample = thirdSampleSlowMoveIn.fresh()
//                .setTangent(Math.toRadians(-90))
//                .splineToLinearHeading(deliverThirdMovePose , Math.toRadians(-315))
//                .endTrajectory();
//
//        Pose2d deliverThirdSampleMoveInPose = new Pose2d(-61.5 + X_OFFSET,-57.5 + Y_OFFSET, Math.toRadians(-315));
//
//        deliverThirdSampleMoveIn = deliverThirdSample.fresh()
//                .splineToLinearHeading(deliverThirdSampleMoveInPose, Math.toRadians(-90))
//                .endTrajectory();
//
//        Pose2d pick4thPose = new Pose2d (7, -62, Math.toRadians(0));
//
//        pick4thMoveIn = dropOffPreload.fresh()
//
//                .setTangent(Math.toRadians(0))
//                .splineToLinearHeading(pick4thPose, Math.toRadians(0))
//                .endTrajectory();
//
//        Pose2d pick4thSlowPose = new Pose2d(21.9, -62, Math.toRadians(0));
//
//        pick4thSlowMoveIn = pick4thMoveIn.fresh()
////                .setTangent(Math.toRadians(0))
//                .splineToLinearHeading(pick4thSlowPose, Math.toRadians(0), new TranslationalVelConstraint(8))
//                .endTrajectory();
//
//        Pose2d dropOff4thPose = new Pose2d(-53.2, -62.5, Math.toRadians(15));
//
//        dropOff4th =drive.actionBuilder(drive.pose)
//                .setTangent(Math.toRadians(0))
////                .strafeTo(new Vector2d(-48, -58))
//                .splineToLinearHeading(dropOff4thPose, Math.toRadians(0), new TranslationalVelConstraint(7))
//                .endTrajectory();
//
//        park = deliverThirdSampleMoveIn.fresh()
//                .setTangent(Math.toRadians(90))
//               // .splineToLinearHeading(new Pose2d(-10,-10,Math.toRadians(180)),Math.toRadians(0))
//                .splineToLinearHeading(new Pose2d(-19, -3, Math.toRadians(180)), Math.toRadians(0), new TranslationalVelConstraint(85))
//                .endTrajectory();
//
////        intakeSubsystem.setDesiredColour(IntakeSubsystem.SampleColour.NEUTRAL);
//        intakeSubsystem.intakePivotDown();
//        transferSubsystem.closeGrippler();
//
//
//        CommandScheduler.getInstance().schedule(
//                new WaitUntilCommand(this::isStarted).andThen(
////                        new ActionCommand(dropOffPreload.build(), new ArraySet<>())
//                    new SequentialCommandGroup(
//
//                            new SequentialCommandGroup(
//                                    new CloseGripplerCommand(transferSubsystem),
//                                            // do the drop off if we have the sample
//                                            new SequentialCommandGroup(
//                                                    new ParallelCommandGroup(
//
//                                                            new DeliveryCommandGroup(intakeSubsystem, transferSubsystem, slidesSubsystem, robotState ),
//                                                          new SequentialCommandGroup(
//                                                                  new WaitCommand(300), //give the slides time to move up
//                                                                  new ActionCommand(dropOffPreload.build(), new ArraySet<>())
//                                                          )
//                                                    ),
//
//                                                    new OpenGripplerCommand(transferSubsystem),
//                                                    new WaitCommand(200)
//
//                                            )
//
//                            ),
//
//                            new ParallelCommandGroup(
//
//                                    new ActionCommand(pick4thMoveIn.build(), new ArraySet<>()),
//                                    new DeliveryResetCommandGroup(intakeSubsystem,transferSubsystem,slidesSubsystem, robotState),
//                                    new SequentialCommandGroup(
//                                            new InstantCommand(intakeSubsystem::intakeSlidesOut), // new IntakeSlidesOutCommand(intakeSubsystem),
//                                            new InstantCommand(intakeSubsystem::intakePivotDown)// new IntakePivotDownCommand(intakeSubsystem, robotState)
//                                    )
//                            ),
//
//                            new SequentialCommandGroup(
//
//                                    new ParallelCommandGroup(
//
//                                            new SequentialCommandGroup(
//                                                    new ColourAwareIntakeCommand(intakeSubsystem).withTimeout(150)
//                                            ),
//                                            new ActionCommand(pick4thSlowMoveIn.build(), new ArraySet<>())
//
//                                    )
//
//
//
//                            ),
//
//                            new IntakeCommandGroup(intakeSubsystem, transferSubsystem, robotState),
////
//                            new SequentialCommandGroup(
//                                    new CloseGripplerCommand(transferSubsystem),
//                                    new IntakeSlidesOutCommand(intakeSubsystem),
//
//                                    // do the drop off if we have the sample
//                                    new SequentialCommandGroup(
//                                            new ParallelCommandGroup(
//                                                    new ActionCommand(dropOff4th.build(), new ArraySet<>()),
//                                                    new WaitCommand(1500),
//                                                    new DeliveryCommandGroup(intakeSubsystem, transferSubsystem, slidesSubsystem, robotState )
//                                            ),
//
//                                            new OpenGripplerCommand(transferSubsystem),
//                                            new WaitCommand(250)
//                                    )
//
//                            ),
//
//
//                            new ParallelCommandGroup(
//
//                                    new ActionCommand(firstSample.build(), new ArraySet<>()),
//                                    new WaitCommand(300),
//                                    new DeliveryResetCommandGroup(intakeSubsystem,transferSubsystem,slidesSubsystem, robotState),
//                                    new SequentialCommandGroup(
//                                          new InstantCommand(intakeSubsystem::intakeSlidesOut), // new IntakeSlidesOutCommand(intakeSubsystem),
//                                          new InstantCommand(intakeSubsystem::intakePivotDown) // new IntakePivotDownCommand(intakeSubsystem, robotState)
//                                    )
//                            ),
//                            new SequentialCommandGroup(
//
//                                    new ParallelCommandGroup(
//                                            new SequentialCommandGroup(
//                                                    new WaitCommand(300),
//                                                    new ColourAwareIntakeCommand(intakeSubsystem).withTimeout(150)
//                                            ),
//                                            new ActionCommand(firstSampleSlowMoveIn.build(), new ArraySet<>())
//                                    )
//
//                            ),
//                            new IntakeCommandGroup(intakeSubsystem, transferSubsystem, robotState),
////
//                            new SequentialCommandGroup(
//                                    new CloseGripplerCommand(transferSubsystem),
//                                    new IntakeSlidesOutCommand(intakeSubsystem),
//
//                                    // do the drop off if we have the sample
//                                    new SequentialCommandGroup(
//                                            new ParallelCommandGroup(
//                                                    new DeliveryCommandGroup(intakeSubsystem, transferSubsystem, slidesSubsystem, robotState )
//                                            ),
//                                            new ActionCommand(firstSampleDeliverIn.build(), new ArraySet<>()),
//                                            new OpenGripplerCommand(transferSubsystem),
//                                            new WaitCommand(150)
//                                    )
//
//                            ),
//                            new ParallelCommandGroup(
//
//                                    new ActionCommand(secondSample.build(), new ArraySet<>()),
//                                    new DeliveryResetCommandGroup(intakeSubsystem,transferSubsystem,slidesSubsystem, robotState),
//                                    new SequentialCommandGroup(
//                                            new InstantCommand(intakeSubsystem::intakeSlidesOut), // new IntakeSlidesOutCommand(intakeSubsystem),
//                                            new InstantCommand(intakeSubsystem::intakePivotDown) // new IntakePivotDownCommand(intakeSubsystem, robotState)
//                                    )
//                            ),
//                            new SequentialCommandGroup(
//
//                                    new ParallelCommandGroup(
//                                        new SequentialCommandGroup(
//                                            new ColourAwareIntakeCommand(intakeSubsystem).withTimeout(150)
//                                        ),
//                                        new ActionCommand(secondSampleSlowMoveIn.build(), new ArraySet<>())
//                                    )
//
//                            ),
//
//
//                            new IntakeCommandGroup(intakeSubsystem, transferSubsystem, robotState),
//
//
//
//                            new SequentialCommandGroup(
//                                    new CloseGripplerCommand(transferSubsystem),
//                                    new IntakeSlidesOutCommand(intakeSubsystem),
//
//                                    // do the drop off if we have the sample
//                                    new SequentialCommandGroup(
//                                            new ParallelCommandGroup(
//
//                                                    new DeliveryCommandGroup(intakeSubsystem, transferSubsystem, slidesSubsystem, robotState )
//                                            ),
//                                            new ActionCommand(deliverSecondSampleMoveIn.build(), new ArraySet<>()),
//                                            new OpenGripplerCommand(transferSubsystem),
//                                            new WaitCommand(250)
//                                    ),
////
////
////                            ),
////
////
//                            new ParallelCommandGroup(
//
//                                    new ActionCommand(thirdSample.build(), new ArraySet<>()),
//                                    new DeliveryResetCommandGroup(intakeSubsystem,transferSubsystem,slidesSubsystem, robotState)
//                            ),
//                            //third sample
//
//
//                            new ParallelCommandGroup(
//                                    new SequentialCommandGroup(
//                                            new IntakeSlidesOutCommand(intakeSubsystem),
//                                            new IntakePivotDownCommand(intakeSubsystem, robotState),
//                                            new ColourAwareIntakeCommand(intakeSubsystem).withTimeout(400)
//                                    ),
//
//                                    new ActionCommand(thirdSampleSlowMoveIn.build(), new ArraySet<>())
//
//                            ),
//                            new IntakeCommandGroup(intakeSubsystem, transferSubsystem, robotState),
//
//                            new SequentialCommandGroup(
//                                    new ParallelCommandGroup(
//
//                                            new DeliveryCommandGroup(intakeSubsystem, transferSubsystem, slidesSubsystem, robotState )
//                                    ),
//                                    new ActionCommand(deliverThirdSampleMoveIn.build(), new ArraySet<>()),
//                                    new OpenGripplerCommand(transferSubsystem),
//                                    new WaitCommand(250)
//                            ),
//                            new DeliveryResetCommandGroup(intakeSubsystem,transferSubsystem,slidesSubsystem, robotState),
//
//                            new ParallelCommandGroup(
//                                    new ActionCommand(park.build(), new ArraySet<>()),
//                                    new InstantCommand(()->{
//                                        new TransferFlipCommand(transferSubsystem);
//
//                                    })
////                                    new DeliveryResetCommandGroup(intakeSubsystem,transferSubsystem,slidesSubsystem, robotState)
//
////                                    new AscentOpenHooksCommand(ascentSubsystem)
//
//                            )
//
//
//
//
//
//                )
//
//                )
//                )
//
//
//        );
//
//    }
//
//}
