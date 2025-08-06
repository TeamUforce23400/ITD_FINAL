package org.firstinspires.ftc.teamcode.opmodes.auto;

import androidx.collection.ArraySet;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.ActionCommand;
import org.firstinspires.ftc.teamcode.commands.ClawCloseCommand;
import org.firstinspires.ftc.teamcode.commands.CloseGripplerCommand;
import org.firstinspires.ftc.teamcode.commands.FireOneShotCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeClawOpenCommand;
import org.firstinspires.ftc.teamcode.commands.IntakePivotDownCommand;
import org.firstinspires.ftc.teamcode.commands.IntakePivotMid;
import org.firstinspires.ftc.teamcode.commands.IntakeSlidesInCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeSlidesOutCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeSlidesOutHalfCommand;
import org.firstinspires.ftc.teamcode.commands.OpenGripplerCommand;
import org.firstinspires.ftc.teamcode.commands.SlidesHighBasketCommand;
import org.firstinspires.ftc.teamcode.commands.SlidesStowCommand;
import org.firstinspires.ftc.teamcode.commands.TransferFlipCommand;
import org.firstinspires.ftc.teamcode.commands.TransferStowCommand;
import org.firstinspires.ftc.teamcode.commands.TurretNormalResetCommand;
import org.firstinspires.ftc.teamcode.commands.groups.DeliveryResetCommandGroup;
import org.firstinspires.ftc.teamcode.commands.groups.RetractCommandGroup;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotStateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlidesSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.opencv.core.Mat;

@Autonomous(name = "BASKET | FOUR | PARK", group = "Autonomous")
public class BasketAutoFourPP extends CommandOpMode {

    private TrajectoryActionBuilder dropOffPreload;
    private TrajectoryActionBuilder predropoff5;
    private TrajectoryActionBuilder firstSample;
    private TrajectoryActionBuilder secondSample;
    private TrajectoryActionBuilder firstSampleDrop;
    private TrajectoryActionBuilder secondSampleDrop;
    private TrajectoryActionBuilder park;
    private TrajectoryActionBuilder parkIntake;
    private TrajectoryActionBuilder dropoff5;
    private TrajectoryActionBuilder parkFinal;
    private TrajectoryActionBuilder parkFinalFinal;

    private IntakeSubsystem intakeSubsystem;
    private TransferSubsystem transferSubsystem;
    private RobotStateSubsystem robotState;
    private SlidesSubsystem slidesSubsystem;

    private static final int X_OFFSET = 0;
    private static final int Y_OFFSET = 0;

    @Override
    public void initialize() {

        //y is forward
        //x is strafe
        intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetry);
        transferSubsystem = new TransferSubsystem(hardwareMap);
        robotState = new RobotStateSubsystem();
        slidesSubsystem = new SlidesSubsystem(hardwareMap, telemetry);

        // Instantiate drive
        MecanumDrive drive = new MecanumDrive(hardwareMap,
                new Pose2d(-48, -68, Math.toRadians(90)));

        // Preload drop-off trajectory
        Vector2d dropOffPose = new Vector2d(-70, -62);
        dropOffPreload = drive.actionBuilder(
                        new Pose2d(-48, -67.7, Math.toRadians(90)))
                .setTangent(Math.toRadians(0))
                .strafeToLinearHeading(dropOffPose, Math.toRadians(67.5), new TranslationalVelConstraint(15))
                .endTrajectory();

        // First sample trajectory
        Vector2d firstSamplePose = new Vector2d(-70, -64);
        firstSample = dropOffPreload.fresh()
                .strafeToLinearHeading(firstSamplePose, Math.toRadians(85.5), new TranslationalVelConstraint(15))
                .endTrajectory();

        Vector2d firstSampleDropPose = new Vector2d(-69.5, -61.5);
        firstSampleDrop = firstSample.fresh()
                .turnTo(Math.toRadians(72))
                .endTrajectory();

        Vector2d secondSamplePose = new Vector2d(-69.5, -65.7);
        secondSample = firstSampleDrop.fresh()
                .turnTo(Math.toRadians(108.8))
                .endTrajectory();

        Vector2d secondSampleDropPose = new Vector2d(-69.5, -61);
        secondSampleDrop = secondSample.fresh()
                .turnTo(Math.toRadians(68))
                .endTrajectory();

                parkIntake = secondSampleDrop.fresh()
                .setTangent(Math.toRadians(0))
               // .splineToLinearHeading(new Pose2d(-10,-10,Math.toRadians(180)),Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(-41, -5), Math.toRadians(0), new TranslationalVelConstraint(100))
                .endTrajectory();

                park = parkIntake.fresh()
                        .setTangent(Math.toRadians(0))
                                .strafeToLinearHeading(new Vector2d(-36, -2), Math.toRadians(0), new TranslationalVelConstraint(100))
                                        .endTrajectory();
        predropoff5 = park.fresh()
                .setTangent(0)
                .strafeToLinearHeading(new Vector2d(-41, -2), Math.toRadians(0))
                .endTrajectory();

                dropoff5 = predropoff5.fresh()
                        .setTangent(0)
                        .strafeToLinearHeading(new Vector2d(-71, -62.7), Math.toRadians(71), new TranslationalVelConstraint(100))
                        .endTrajectory();

        parkFinal = dropoff5.fresh()
                .setTangent(Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(-30, 1), Math.toRadians(180), new TranslationalVelConstraint(120))
                .endTrajectory();

        parkFinalFinal = parkFinal.fresh()
                .setTangent(Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(-36, -5), Math.toRadians(180), new TranslationalVelConstraint(120))
                .endTrajectory();




        // Ensure gripper is closed before starting
        transferSubsystem.closeGrippler();

        // Schedule commands sequentially
        CommandScheduler.getInstance().schedule(
                new WaitUntilCommand(this::isStarted)
                        .andThen(
                                new SequentialCommandGroup(
                                        // Drop off preload
                                        new SequentialCommandGroup(
                                                new CloseGripplerCommand(transferSubsystem),
                                                new ParallelCommandGroup(
                                                        new SlidesHighBasketCommand(slidesSubsystem),
                                                        new TransferFlipCommand(transferSubsystem),
                                                        new ActionCommand(dropOffPreload.build(), new ArraySet<>()),
                                                        new IntakeSlidesOutCommand(intakeSubsystem)

                                                ),
                                                new InstantCommand(transferSubsystem::openGrippler),
                                                new InstantCommand(intakeSubsystem::intakePivotMid)

                                        ),

                                        // Intake and stow
                                        new SequentialCommandGroup(
//                                                new InstantCommand(transferSubsystem::openGrippler).withTimeout(100),
//                                                new InstantCommand(intakeSubsystem::intakePivotMid),
                                                new IntakePivotDownCommand(intakeSubsystem, robotState),
                                                new WaitCommand(150),
                                                new ParallelCommandGroup(
                                                        new TransferStowCommand(transferSubsystem),
                                                        new SlidesStowCommand(slidesSubsystem)

                                                ),
                                                new WaitCommand(100),
                                                new ClawCloseCommand(intakeSubsystem),
                                                new ParallelCommandGroup(
                                                        new SlidesStowCommand(slidesSubsystem),
                                                        new RetractCommandGroup(transferSubsystem, robotState, intakeSubsystem).withTimeout(1800)
                                                )
                                        ),

                                        // Transfer and prepare for sample
                                        new SequentialCommandGroup(
                                                new InstantCommand(transferSubsystem::closeGrippler),
                                                new WaitCommand(200),
                                                new IntakeClawOpenCommand(intakeSubsystem),
                                                new InstantCommand(intakeSubsystem::intakeSlidesFrontTransfer),
                                                new ParallelCommandGroup(
                                                        new InstantCommand(slidesSubsystem::highBasket),
                                                        new InstantCommand(transferSubsystem::flipTransfer),
                                                        new InstantCommand(intakeSubsystem::intakeSlidesOut),
                                                        new ActionCommand(firstSample.build(), new ArraySet<>())

                                                ),
                                                new TurretNormalResetCommand(intakeSubsystem),
                                                new InstantCommand(intakeSubsystem::intakeClawOpen),
                                                new WaitCommand(200),
                                                new InstantCommand(intakeSubsystem::intakePivotMid),
                                                new WaitCommand(100),
                                                new OpenGripplerCommand(transferSubsystem),
                                                new ParallelCommandGroup(
                                                        new TransferStowCommand(transferSubsystem),
                                                        new SlidesStowCommand(slidesSubsystem),
                                                        new IntakePivotDownCommand(intakeSubsystem, robotState)

                                                ),
                                                new WaitCommand(300),
                                                new ClawCloseCommand(intakeSubsystem),
                                                new WaitCommand(300),
                                                new RetractCommandGroup(transferSubsystem, robotState, intakeSubsystem).withTimeout(1800)
                                        ),

                                        new SequentialCommandGroup(
                                                new ParallelCommandGroup(
                                                        new InstantCommand(transferSubsystem::closeGrippler),
                                                        new ActionCommand(firstSampleDrop.build(), new ArraySet<>())

                                                ),

                                                new WaitCommand(300),
                                                new IntakeClawOpenCommand(intakeSubsystem),
                                                new InstantCommand(intakeSubsystem::intakeSlidesFrontTransfer),
                                                new ParallelCommandGroup(
                                                        new InstantCommand(slidesSubsystem::highBasket),
                                                        new InstantCommand(transferSubsystem::flipTransfer),
                                                        new InstantCommand(intakeSubsystem::intakeSlidesOut)

                                                ),
                                                new TurretNormalResetCommand(intakeSubsystem),
                                                new InstantCommand(intakeSubsystem::intakeClawOpen),
                                                new WaitCommand(200),
                                                new InstantCommand(intakeSubsystem::intakePivotMid),
                                                new WaitCommand(600),
                                                new OpenGripplerCommand(transferSubsystem),
                                                new ActionCommand(secondSample.build(), new ArraySet<>()),
                                                new ParallelCommandGroup(
                                                        new TransferStowCommand(transferSubsystem),
                                                        new SlidesStowCommand(slidesSubsystem),
                                                        new WaitCommand(200),
                                                        new IntakePivotDownCommand(intakeSubsystem, robotState)

                                                ),
                                                new WaitCommand(500),
                                                new ClawCloseCommand(intakeSubsystem),
                                                        new WaitCommand(300),
                                                        new RetractCommandGroup(transferSubsystem, robotState, intakeSubsystem).withTimeout(1800)

                                        ),
                                        new SequentialCommandGroup(
                                                new WaitCommand(300),
                                                new ParallelCommandGroup(
                                                        new InstantCommand(transferSubsystem::closeGrippler),
                                                        new ActionCommand(secondSampleDrop.build(), new ArraySet<>())
                                                ),

                                                new WaitCommand(200),
                                                new IntakeClawOpenCommand(intakeSubsystem),
                                                new InstantCommand(intakeSubsystem::intakeSlidesFrontTransfer),
                                                new ParallelCommandGroup(
                                                        new InstantCommand(slidesSubsystem::highBasket),
                                                        new InstantCommand(transferSubsystem::flipTransfer)
//                                                        new InstantCommand(intakeSubsystem::intakeSlidesOut)

                                                ),
                                                new TurretNormalResetCommand(intakeSubsystem),
                                                new InstantCommand(intakeSubsystem::intakeClawOpen),

                                                new WaitCommand(1000),
                                                new OpenGripplerCommand(transferSubsystem),
                                                new ParallelCommandGroup(
//                                                        new TransferStowCommand(transferSubsystem),
                                                        new SlidesStowCommand(slidesSubsystem)

                                                )
//                                                new ClawCloseCommand(intakeSubsystem),
//                                                new WaitCommand(300),
//                                                new RetractCommandGroup(transferSubsystem, robotState, intakeSubsystem).withTimeout(1800)
                                        ),

                                        new SequentialCommandGroup(
                                                new ParallelCommandGroup(
                                                        new ActionCommand(parkIntake.build(), new ArraySet<>()),
                                                        new InstantCommand(intakeSubsystem::intakeClawMidBase)
                                                ),


                                                new IntakeSlidesOutHalfCommand(intakeSubsystem),
                                                new ActionCommand(park.build(), new ArraySet<>()),
                                                new WaitCommand(100),
                                                new InstantCommand(intakeSubsystem::intakePivotDown),
                                                new WaitCommand(500),
                                                new ClawCloseCommand(intakeSubsystem),

                                                new ParallelCommandGroup(
                                                        new RetractCommandGroup(transferSubsystem, robotState, intakeSubsystem)).withTimeout(1800),
                                                        new WaitCommand(200),
                                                        new ActionCommand(predropoff5.build(), new ArraySet<>()),
                                                new CloseGripplerCommand(transferSubsystem),
                                                new IntakeClawOpenCommand(intakeSubsystem),
                                                new InstantCommand(intakeSubsystem::intakeSlidesFrontTransfer),
                                                new ParallelCommandGroup(
                                                        new ActionCommand(dropoff5.build(), new ArraySet<>()),
                                                        new SlidesHighBasketCommand(slidesSubsystem),
                                                        new TransferFlipCommand(transferSubsystem),
                                                        new TurretNormalResetCommand(intakeSubsystem),
                                                        new InstantCommand(intakeSubsystem::intakeClawOpen)
                                                ),
                                                new WaitCommand(500),
                                                new OpenGripplerCommand(transferSubsystem),
                                                new WaitCommand(200),
                                                new ParallelCommandGroup(
                                                        new ActionCommand(parkFinal.build(), new ArraySet<>()),
                                                        new SlidesStowCommand(slidesSubsystem)
                                                )
//                                                new ActionCommand(parkFinalFinal.build(), new ArraySet<>())

                                )))

//                                        new SequentialCommandGroup(
//                                                new ParallelCommandGroup(
//                                                        new InstantCommand(transferSubsystem::closeGrippler),
//                                                        new ActionCommand(firstSampleDrop.build(), new ArraySet<>())
//                                                ),
//
//                                                new WaitCommand(200),
//                                                new IntakeClawOpenCommand(intakeSubsystem),
//                                                new InstantCommand(intakeSubsystem::intakeSlidesFrontTransfer),
//                                                new ParallelCommandGroup(
//                                                        new InstantCommand(slidesSubsystem::highBasket),
//                                                        new InstantCommand(transferSubsystem::flipTransfer),
//                                                        new InstantCommand(intakeSubsystem::intakeSlidesOut)
////                                                        new ActionCommand(secondSample.build(), new ArraySet<>())
//
//                                                ),
//                                                new TurretNormalResetCommand(intakeSubsystem),
//                                                new InstantCommand(intakeSubsystem::intakeClawOpen),
//                                                new WaitCommand(200),
//                                                new InstantCommand(intakeSubsystem::intakePivotMid)
////                                                new ClawCloseCommand(intakeSubsystem),
////                                                new WaitCommand(300),
////                                                new RetractCommandGroup(transferSubsystem, robotState, intakeSubsystem).withTimeout(1800)
//
//                                        )
                    //
//                                        // Drop first sample
//                                        new SequentialCommandGroup(
//                                                new ActionCommand(firstSample.build(), new ArraySet<>()),
//                                                new ParallelCommandGroup(
//                                                        new InstantCommand(transferSubsystem::openGrippler).withTimeout(100)
//                                                ),
//                                                new ClawCloseCommand(intakeSubsystem),
//                                                new ParallelCommandGroup(
//                                                        new TransferStowCommand(transferSubsystem),
//                                                        new SlidesStowCommand(slidesSubsystem)
////                                                )
////                                        )



        );
    }
}



//package org.firstinspires.ftc.teamcode.opmodes.auto;
//import androidx.collection.ArraySet;
//
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
//import com.acmerobotics.roadrunner.TranslationalVelConstraint;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.arcrobotics.ftclib.command.CommandOpMode;
//import com.arcrobotics.ftclib.command.CommandScheduler;
//import com.arcrobotics.ftclib.command.InstantCommand;
//import com.arcrobotics.ftclib.command.ParallelCommandGroup;
//import com.arcrobotics.ftclib.command.SequentialCommandGroup;
//import com.arcrobotics.ftclib.command.WaitCommand;
//import com.arcrobotics.ftclib.command.WaitUntilCommand;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.acmerobotics.roadrunner.Pose2d;
//
//
//import org.firstinspires.ftc.teamcode.commands.ClawCloseCommand;
//import org.firstinspires.ftc.teamcode.commands.IntakeClawOpenCommand;
//import org.firstinspires.ftc.teamcode.commands.IntakePivotMid;
//import org.firstinspires.ftc.teamcode.commands.IntakeSlidesInCommand;
//import org.firstinspires.ftc.teamcode.commands.SlidesHighBasketCommand;
//import org.firstinspires.ftc.teamcode.commands.SlidesStowCommand;
//import org.firstinspires.ftc.teamcode.commands.TransferStowCommand;
//import org.firstinspires.ftc.teamcode.commands.TurretNormalResetCommand;
//import org.firstinspires.ftc.teamcode.commands.groups.RetractCommandGroup;
//import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
//import org.firstinspires.ftc.teamcode.commands.ActionCommand;
//import org.firstinspires.ftc.teamcode.commands.CloseGripplerCommand;
////import org.firstinspires.ftc.teamcode.commands.ColourAwareIntakeCommand;
//import org.firstinspires.ftc.teamcode.commands.IntakePivotDownCommand;
//import org.firstinspires.ftc.teamcode.commands.IntakeSlidesOutCommand;
//import org.firstinspires.ftc.teamcode.commands.OpenGripplerCommand;
//import org.firstinspires.ftc.teamcode.commands.TransferFlipCommand;
//import org.firstinspires.ftc.teamcode.commands.groups.DeliveryCommandGroup;
//import org.firstinspires.ftc.teamcode.commands.groups.DeliveryResetCommandGroup;
////import org.firstinspires.ftc.teamcode.commands.groups.IntakeCommandGroup;
//import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.RobotStateSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.SlidesSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;
//import com.acmerobotics.roadrunner.Pose2d;
//
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
//                new Pose2d(-48, -70, Math.toRadians(90)));
//
//        //pose to the submersible wall
//        Pose2d dropOffPose = new Pose2d(-67.5, -64, Math.toRadians(75));
//
//        dropOffPreload = drive.actionBuilder (new Pose2d(-48, -70, Math.toRadians(90)))
//                .setTangent(Math.toRadians(0))
////                .strafeTo(new Vector2d(-48, -58))
//                .splineToLinearHeading(dropOffPose, Math.toRadians(78))
//                .endTrajectory();
//
//
//
//
//
//
//
//        Pose2d firstSamplePose = new Pose2d(-67.5, -64, Math.toRadians(90));
//
//
//        firstSample = dropOffPreload.fresh()
//                .splineToLinearHeading(firstSamplePose, Math.toRadians(90))
//                .endTrajectory();
////
////        Pose2d firstSampleSlowMoveInPose = new Pose2d(-53.8, -43.9, Math.toRadians(90));
////
////        firstSampleSlowMoveIn = firstSample.fresh()
////                .splineToLinearHeading(firstSampleSlowMoveInPose, Math.toRadians(100), new TranslationalVelConstraint(7))
////                .endTrajectory();
////
////
////        Pose2d firstSampleDeliverPose = new Pose2d(-61 + X_OFFSET,-53.3 + Y_OFFSET,Math.toRadians(-315));
////
////
////        firstSampleDeliver = firstSampleSlowMoveIn.fresh()
////                .splineToLinearHeading(firstSampleDeliverPose, Math.toRadians(90))
////                .endTrajectory();
////
////        Pose2d deliverFirstSampleMoveInPose = new Pose2d(-63 + X_OFFSET,-56 + Y_OFFSET, Math.toRadians(-315));
////
////        firstSampleDeliverIn = firstSampleDeliver.fresh()
////                .splineToLinearHeading(deliverFirstSampleMoveInPose, Math.toRadians(90))
////                .endTrajectory();
////
////        Pose2d secondSamplePose = new Pose2d(-66.2 + X_OFFSET,-59.5 + Y_OFFSET,Math.toRadians(90));
////
////
////        secondSample = firstSampleDeliverIn.fresh()
////                .setTangent(Math.toRadians(90))
////                .splineToLinearHeading(secondSamplePose,Math.toRadians(90))
////                .endTrajectory();
////
////        Pose2d secondSampleMoveInPose = new Pose2d(-66.7 + X_OFFSET,-43.5 + Y_OFFSET,Math.toRadians(90));
////
////
////        secondSampleSlowMoveIn = secondSample.fresh()
////                .splineToLinearHeading(secondSampleMoveInPose, Math.toRadians(90), new TranslationalVelConstraint(8))
////                .endTrajectory();
////
////        Pose2d deliverSecondSamplePose = new Pose2d(-60 + X_OFFSET,-55 + Y_OFFSET,Math.toRadians(-315));
////
////        deliverSecondSample = secondSampleSlowMoveIn.fresh()
////                .setTangent(Math.toRadians(90))
////                .splineToLinearHeading(deliverSecondSamplePose,Math.toRadians(90), new TranslationalVelConstraint(11))
////                .endTrajectory();
////
////        Pose2d deliverSecondSampleMoveInPose = new Pose2d(-64 + X_OFFSET,-55 + Y_OFFSET, Math.toRadians(-315));
////
////
////        deliverSecondSampleMoveIn = deliverSecondSample.fresh()
////                .splineToLinearHeading(deliverSecondSampleMoveInPose, Math.toRadians(90))
////                .endTrajectory();
////
////        Pose2d thirdSamplePose = new Pose2d(-58 + X_OFFSET,-50 + Y_OFFSET,Math.toRadians(-235));
////
////        thirdSample = deliverSecondSampleMoveIn.fresh()
////                .setTangent(Math.toRadians(140))
////                .splineToLinearHeading(thirdSamplePose,Math.toRadians(140), new TranslationalVelConstraint(9))
////                .endTrajectory();
////
////        Pose2d thirdSampleMoveInPose = new Pose2d(-62.6  + X_OFFSET,-43.0 + Y_OFFSET,Math.toRadians(-235));
////
////        thirdSampleSlowMoveIn = thirdSample.fresh()
////                .splineToLinearHeading(thirdSampleMoveInPose, Math.toRadians(90), new TranslationalVelConstraint(8))
////                .endTrajectory();
////
////        Pose2d deliverThirdMovePose = new Pose2d(-60 + X_OFFSET,-53 + Y_OFFSET,Math.toRadians(-315));
////
////        deliverThirdSample = thirdSampleSlowMoveIn.fresh()
////                .setTangent(Math.toRadians(-90))
////                .splineToLinearHeading(deliverThirdMovePose , Math.toRadians(-315))
////                .endTrajectory();
////
////        Pose2d deliverThirdSampleMoveInPose = new Pose2d(-61.5 + X_OFFSET,-57.5 + Y_OFFSET, Math.toRadians(-315));
////
////        deliverThirdSampleMoveIn = deliverThirdSample.fresh()
////                .splineToLinearHeading(deliverThirdSampleMoveInPose, Math.toRadians(-90))
////                .endTrajectory();
////
////        Pose2d pick4thPose = new Pose2d (7, -62, Math.toRadians(0));
////
////        pick4thMoveIn = dropOffPreload.fresh()
////
////                .setTangent(Math.toRadians(0))
////                .splineToLinearHeading(pick4thPose, Math.toRadians(0))
////                .endTrajectory();
////
////        Pose2d pick4thSlowPose = new Pose2d(21.9, -62, Math.toRadians(0));
////
////        pick4thSlowMoveIn = pick4thMoveIn.fresh()
//////                .setTangent(Math.toRadians(0))
////                .splineToLinearHeading(pick4thSlowPose, Math.toRadians(0), new TranslationalVelConstraint(8))
////                .endTrajectory();
////
////        Pose2d dropOff4thPose = new Pose2d(-53.2, -62.5, Math.toRadians(15));
////
////        dropOff4th =drive.actionBuilder(drive.)
////                .setTangent(Math.toRadians(0))
//////                .strafeTo(new Vector2d(-48, -58))
////                .splineToLinearHeading(dropOff4thPose, Math.toRadians(0), new TranslationalVelConstraint(7))
////                .endTrajectory();
////
////        park = deliverThirdSampleMoveIn.fresh()
////                .setTangent(Math.toRadians(90))
////               // .splineToLinearHeading(new Pose2d(-10,-10,Math.toRadians(180)),Math.toRadians(0))
////                .splineToLinearHeading(new Pose2d(-19, -3, Math.toRadians(180)), Math.toRadians(0), new TranslationalVelConstraint(85))
////                .endTrajectory();
//
////        intakeSubsystem.setDesiredColour(IntakeSubsystem.SampleColour.NEUTRAL);
////        intakeSubsystem.intakePivotDown();
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
//                                                            new SlidesHighBasketCommand(slidesSubsystem), new TransferFlipCommand(transferSubsystem),
//                                                          new SequentialCommandGroup(
//                                                                  new ActionCommand(dropOffPreload.build(), new ArraySet<>())
//                                                          ),
//
//                                                            new IntakeSlidesOutCommand(intakeSubsystem)
//
//                                                    ),
//                                                    new InstantCommand(transferSubsystem::openGrippler)),
//                                                     new InstantCommand(intakeSubsystem::intakePivotMid)
//
//
//
//                                            )
//
//                            ),
//
//
//                            new SequentialCommandGroup(
//                                    new IntakePivotDownCommand(intakeSubsystem, robotState),
//                                    new ParallelCommandGroup(
//                                            new TransferStowCommand(transferSubsystem),
//                                            new SlidesStowCommand(slidesSubsystem),
//                                            new WaitCommand(200),
//                                            new ClawCloseCommand(intakeSubsystem)
//
//
//                                    ),
////
////
//////                                    new ClawCloseCommand(intakeSubsystem),
//                                    new ParallelCommandGroup(
//                                            new SlidesStowCommand(slidesSubsystem),
//                                            new RetractCommandGroup(transferSubsystem, robotState, intakeSubsystem).withTimeout(500)
//                                    )
//                            ),
////
//                            new SequentialCommandGroup(
//                                    new InstantCommand(transferSubsystem::closeGrippler),
//                                    //Transfer & Slides
//                                    new WaitCommand(200),
//                                    new IntakeClawOpenCommand(intakeSubsystem),
//                                    new InstantCommand(()-> intakeSubsystem.intakeSlidesFrontTransfer()),
//                                    new InstantCommand(()-> transferSubsystem.flipTransfer()),
//                                    new TurretNormalResetCommand(intakeSubsystem),
//                                    new InstantCommand(intakeSubsystem::intakeClawOpen),
//                                    new IntakeSlidesInCommand(intakeSubsystem, transferSubsystem)
//
//                                    new WaitCommand(200),
//
//                                    new ParallelCommandGroup(
//
//                                            new IntakeSlidesOutCommand(intakeSubsystem),
//                                            new SlidesHighBasketCommand(slidesSubsystem)
//
//                                    ),
//
//
//                                    new InstantCommand(intakeSubsystem::intakePivotMid),
//
//
//
//                                    new ActionCommand(firstSample.build(), new ArraySet<>()),
//                                    new ParallelCommandGroup(
//                                            new InstantCommand(transferSubsystem::openGrippler).withTimeout(100)),
//                                            new ClawCloseCommand(intakeSubsystem)
//
//                                    ),
//
//                                    new ParallelCommandGroup(
//                                            new TransferStowCommand(transferSubsystem),
//                                            new SlidesStowCommand(slidesSubsystem)
//
//
//                                    ),
////
////                                    new RetractCommandGroup(transferSubsystem, robotState, intakeSubsystem)
////
////
//
//
//                            ),
//
//////
////                            )
////
////                            new SequentialCommandGroup(
////
////                                    new ParallelCommandGroup(
////
////                                            new SequentialCommandGroup(
//////                                                    new ColourAwareIntakeCommand(intakeSubsystem).withTimeout(150)
////                                            ),
////                                            new ActionCommand(pick4thSlowMoveIn.build(), new ArraySet<>())
////
////                                    )
////
////
////
////                            ),
//
////                            new IntakeCommandGroup(intakeSubsystem, transferSubsystem, robotState),
//////
////
////
//                            new ParallelCommandGroup(
//
//                                    new ActionCommand(firstSample.build(), new ArraySet<>()),
//                                    new WaitCommand(300),
//
//                                    new ParallelCommandGroup(
//                                            new TransferStowCommand(transferSubsystem),
//                                            new SlidesStowCommand(slidesSubsystem)
//                                    ),
//                                    new SequentialCommandGroup(
//                                          new InstantCommand(intakeSubsystem::intakeSlidesOut), // new IntakeSlidesOutCommand(intakeSubsystem),
//                                          new InstantCommand(intakeSubsystem::intakePivotDown) // new IntakePivotDownCommand(intakeSubsystem, robotState)
//                                    )
//                            )
////                            new SequentialCommandGroup(
////
////                                    new ParallelCommandGroup(
////                                            new SequentialCommandGroup(
////                                                    new WaitCommand(300)
//////                                                    new ColourAwareIntakeCommand(intakeSubsystem).withTimeout(150)
////                                            ),
////                                            new ActionCommand(firstSampleSlowMoveIn.build(), new ArraySet<>())
////                                    )
////
////                            )
//////                            new IntakeCommandGroup(intakeSubsystem, transferSubsystem, robotState),
//////
////                            new SequentialCommandGroup(
////                                    new CloseGripplerCommand(transferSubsystem),
////                                    new IntakeSlidesOutCommand(intakeSubsystem),
////
////                                    // do the drop off if we have the sample
////                                    new SequentialCommandGroup(
////                                            new ParallelCommandGroup(
////                                                    new DeliveryCommandGroup(intakeSubsystem, transferSubsystem, slidesSubsystem, robotState )
////                                            ),
////                                            new ActionCommand(firstSampleDeliverIn.build(), new ArraySet<>()),
////                                            new OpenGripplerCommand(transferSubsystem),
////                                            new WaitCommand(150)
////                                    )
////
////                            ),
////                            new ParallelCommandGroup(
////
////                                    new ActionCommand(secondSample.build(), new ArraySet<>()),
////                                    new DeliveryResetCommandGroup(intakeSubsystem,transferSubsystem,slidesSubsystem, robotState),
////                                    new SequentialCommandGroup(
////                                            new InstantCommand(intakeSubsystem::intakeSlidesOut), // new IntakeSlidesOutCommand(intakeSubsystem),
////                                            new InstantCommand(intakeSubsystem::intakePivotDown) // new IntakePivotDownCommand(intakeSubsystem, robotState)
////                                    )
////                            ),
////                            new SequentialCommandGroup(
////
////                                    new ParallelCommandGroup(
////                                        new SequentialCommandGroup(
//////                                            new ColourAwareIntakeCommand(intakeSubsystem).withTimeout(150)
////                                        ),
////                                        new ActionCommand(secondSampleSlowMoveIn.build(), new ArraySet<>())
////                                    )
////
////                            ),
////
////
//////                            new IntakeCommandGroup(intakeSubsystem, transferSubsystem, robotState),
////
////
////
////                            new SequentialCommandGroup(
////                                    new CloseGripplerCommand(transferSubsystem),
////                                    new IntakeSlidesOutCommand(intakeSubsystem),
////
////                                    // do the drop off if we have the sample
////                                    new SequentialCommandGroup(
////                                            new ParallelCommandGroup(
////
////                                                    new DeliveryCommandGroup(intakeSubsystem, transferSubsystem, slidesSubsystem, robotState )
////                                            ),
////                                            new ActionCommand(deliverSecondSampleMoveIn.build(), new ArraySet<>()),
////                                            new OpenGripplerCommand(transferSubsystem),
////                                            new WaitCommand(250)
////                                    ),
//////
//////
//////                            ),
//////
//////
////                            new ParallelCommandGroup(
////
////                                    new ActionCommand(thirdSample.build(), new ArraySet<>()),
////                                    new DeliveryResetCommandGroup(intakeSubsystem,transferSubsystem,slidesSubsystem, robotState)
////                            ),
////                            //third sample
////
////
////                            new ParallelCommandGroup(
////                                    new SequentialCommandGroup(
////                                            new IntakeSlidesOutCommand(intakeSubsystem),
////                                            new IntakePivotDownCommand(intakeSubsystem, robotState)
//////                                            new ColourAwareIntakeCommand(intakeSubsystem).withTimeout(400)
////                                    ),
////
////                                    new ActionCommand(thirdSampleSlowMoveIn.build(), new ArraySet<>())
////
////                            ),
//////                            new IntakeCommandGroup(intakeSubsystem, transferSubsystem, robotState),
////
////                            new SequentialCommandGroup(
////                                    new ParallelCommandGroup(
////
////                                            new DeliveryCommandGroup(intakeSubsystem, transferSubsystem, slidesSubsystem, robotState )
////                                    ),
////                                    new ActionCommand(deliverThirdSampleMoveIn.build(), new ArraySet<>()),
////                                    new OpenGripplerCommand(transferSubsystem),
////                                    new WaitCommand(250)
////                            ),
////                            new DeliveryResetCommandGroup(intakeSubsystem,transferSubsystem,slidesSubsystem, robotState),
////
////                            new ParallelCommandGroup(
////                                    new ActionCommand(park.build(), new ArraySet<>()),
////                                    new InstantCommand(()->{
////                                        new TransferFlipCommand(transferSubsystem);
////
////                                    })
//////                                    new DeliveryResetCommandGroup(intakeSubsystem,transferSubsystem,slidesSubsystem, robotState)
////
//////                                    new AscentOpenHooksCommand(ascentSubsystem)
////
////                            )
//
//
//        );
//
//    }
//
//}
