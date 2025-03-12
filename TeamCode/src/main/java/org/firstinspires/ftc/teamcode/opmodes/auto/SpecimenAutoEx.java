package org.firstinspires.ftc.teamcode.opmodes.auto;
import androidx.collection.ArraySet;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
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
import org.firstinspires.ftc.teamcode.commands.ColourAwareIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeOffCommand;
import org.firstinspires.ftc.teamcode.commands.IntakePivotDownCommand;
import org.firstinspires.ftc.teamcode.commands.IntakePivotUpCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeSlidesInCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeSlidesOutCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeSlidesOutHalfCommand;
import org.firstinspires.ftc.teamcode.commands.OpenGripplerCommand;
import org.firstinspires.ftc.teamcode.commands.OuttakeOnCommand;
import org.firstinspires.ftc.teamcode.commands.SlidesStowCommand;
import org.firstinspires.ftc.teamcode.commands.TransferFlipCommand;
import org.firstinspires.ftc.teamcode.commands.TransferStowCommand;
import org.firstinspires.ftc.teamcode.commands.groups.AutoSpecimenCommandGroup;
import org.firstinspires.ftc.teamcode.commands.groups.DeliveryResetCommandGroup;
import org.firstinspires.ftc.teamcode.commands.groups.IntakeCommandGroup;
import org.firstinspires.ftc.teamcode.subsystems.AscentSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotStateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlidesSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;
import org.firstinspires.ftc.teamcode.utils.OTOSDrive;
import org.firstinspires.ftc.teamcode.utils.PoseStorage;

@Autonomous(name = "[OLD] Specimen Ex", group = "Autonomous")
@Disabled
public class SpecimenAutoEx extends CommandOpMode {

    Action dropOffPreload;
    Action firstSample;

    Action firstSampleSlowMoveIn;

    Action firstHumanMove;


    Action secondSample;

    Action secondSampleMoveIn;

    Action secondSampleHuman;

    Action firstInteractionWait;

    Action firstInteractionMoveIn;

    Action humanSlamOne;

    Action humanInteractionTwo;

    Action humanInteractionTwoMoveIn;

    Action humanSlamTwo;

    Action humanInteractionThree;

    Action park;
    IntakeSubsystem intakeSubsystem;

    TransferSubsystem transferSubsystem;

    RobotStateSubsystem robotState;

    SlidesSubsystem slidesSubsystem;

    AscentSubsystem ascentSubsystem;

    @Override
    public void initialize() {

        intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetry);
        transferSubsystem = new TransferSubsystem(hardwareMap);
        robotState = new RobotStateSubsystem();
        slidesSubsystem = new SlidesSubsystem(hardwareMap, telemetry);
        ascentSubsystem = new AscentSubsystem(hardwareMap);


        // instantiate your MecanumDrive at a particular pose.
        OTOSDrive drive = new OTOSDrive(hardwareMap,
                new Pose2d(3.5, -64, Math.toRadians(-90)));

        //pose to the submersible wall
        Pose2d dropOffPose = new Pose2d(-3.5, -32.5, Math.toRadians(-90));

        dropOffPreload = drive.actionBuilder(drive.pose)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(dropOffPose, Math.toRadians(90))
                .build();

        //pose to the first sample
        Pose2d firstSamplePose = new Pose2d(32,-60,Math.toRadians(55));

        firstSample = drive.actionBuilder(dropOffPose)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(firstSamplePose,Math.toRadians(90))
                .build();

        Pose2d sampleOneMoveInPose = new Pose2d(37, -55, Math.toRadians(55));

        firstSampleSlowMoveIn = drive.actionBuilder(firstSamplePose)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(sampleOneMoveInPose, Math.toRadians(55))
                .build();

        Pose2d humanDropOnePose = new Pose2d(37, -54, Math.toRadians(-45));

        firstHumanMove = drive.actionBuilder(sampleOneMoveInPose)
                .splineToLinearHeading(humanDropOnePose, Math.toRadians(90))
                .build();

        Pose2d sampleTwo = new Pose2d(43, -55, Math.toRadians(53));

        secondSample = drive.actionBuilder(humanDropOnePose)
                .splineToLinearHeading(sampleTwo, Math.toRadians(53))
                .build();


        Pose2d sampleTwoMoveInPose = new Pose2d(45, -50, Math.toRadians(53));

        secondSampleMoveIn = drive.actionBuilder(sampleTwo)
                .splineToLinearHeading(sampleTwoMoveInPose, Math.toRadians(55))
                .build();

        Pose2d humanDropTwoPose = new Pose2d(37, -54, Math.toRadians(-45));


        secondSampleHuman = drive.actionBuilder(sampleTwoMoveInPose)
                .splineToLinearHeading(humanDropTwoPose, Math.toRadians(-45))
                .build();

        Pose2d waitSpot1Pose = new Pose2d(33, -50, Math.toRadians(-40));

        firstInteractionWait = drive.actionBuilder(sampleOneMoveInPose)//sampleTwoMoveInPose)
                .splineToLinearHeading(waitSpot1Pose, Math.toRadians(-40))
                .build();

        Pose2d interactionMoveInPose = new Pose2d(40, -50, Math.toRadians(-40));

        firstInteractionMoveIn = drive.actionBuilder(waitSpot1Pose)
                .splineToLinearHeading(interactionMoveInPose, Math.toRadians(-40),  new TranslationalVelConstraint(4))
                .build();

        Pose2d firstHumanSlam = new Pose2d(-3.5, -32.5, Math.toRadians(-90));

        humanSlamOne = drive.actionBuilder(interactionMoveInPose)
                .splineToLinearHeading(firstHumanSlam, Math.toRadians(90))
                .build();

        Pose2d interactionTwoPose = new Pose2d(30,-40, Math.toRadians(-40));

        humanInteractionTwo = drive.actionBuilder(firstHumanSlam)
                .splineToLinearHeading(interactionTwoPose, Math.toRadians(-40))
                .build();

        Pose2d interactionTwoMoveInPose = new Pose2d(34,-40, Math.toRadians(-40));

        humanInteractionTwoMoveIn = drive.actionBuilder(interactionTwoPose)
                .splineToLinearHeading(interactionTwoMoveInPose, Math.toRadians(-40), new TranslationalVelConstraint(4))
                .build();

        Pose2d secondHumanSlam = new Pose2d(-3.5, -32.5, Math.toRadians(-90));

        humanSlamTwo = drive.actionBuilder(interactionTwoMoveInPose)
                .splineToLinearHeading(secondHumanSlam, Math.toRadians(-90))
                .build();

        Pose2d interactionThreePose = new Pose2d(34,-40, Math.toRadians(-40));

        humanInteractionThree = drive.actionBuilder(secondHumanSlam)
                .splineToLinearHeading(interactionThreePose, Math.toRadians(-40))
                .build();


        park = drive.actionBuilder(firstSamplePose)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(55,-65,Math.toRadians(90)),Math.toRadians(0))
                .build();

        intakeSubsystem.setDesiredColour(IntakeSubsystem.SampleColour.AUTO_ANY);
        intakeSubsystem.intakePivotDown();
        transferSubsystem.closeGrippler();


        CommandScheduler.getInstance().schedule(
                new WaitUntilCommand(this::isStarted).andThen(
                    new SequentialCommandGroup(

                            new ActionCommand(dropOffPreload, new ArraySet<>()),

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


                            new ParallelCommandGroup(

                                    new ActionCommand(firstSample, new ArraySet<>()),
                                    new DeliveryResetCommandGroup(intakeSubsystem,transferSubsystem,slidesSubsystem, robotState)
                            ),

                            //ready to move the first sample to the observation zone

                            new SequentialCommandGroup(
                                    new IntakeSlidesOutCommand(intakeSubsystem),
                                    new IntakePivotDownCommand(intakeSubsystem, robotState)//,
                            ),
                            new ParallelCommandGroup(
                                    new SequentialCommandGroup(
                                            new WaitCommand(300), //give time for the movement to start
                                            new ColourAwareIntakeCommand(intakeSubsystem).withTimeout(2000)
                                    ),
                                    new ActionCommand(firstSampleSlowMoveIn, new ArraySet<>())
                            ),

                            new ConditionalCommand(
                                    new AutoSpecimenCommandGroup(intakeSubsystem, transferSubsystem, robotState),
                                    new SequentialCommandGroup(
                                            new IntakeOffCommand(intakeSubsystem), //just retracting with no sample - do nothing
                                            new IntakeSlidesInCommand(intakeSubsystem, transferSubsystem).withTimeout(500)
                                    ),
                                    () -> true// intakeSubsystem.hasItemInIntake()
                            ),

                            new ParallelCommandGroup(
                                new ActionCommand(firstHumanMove, new ArraySet<>()),
                                new IntakeSlidesOutHalfCommand(intakeSubsystem)
                            ),
                            new OuttakeOnCommand(intakeSubsystem),
                            new WaitCommand(500),
                            new IntakeOffCommand(intakeSubsystem),
                            new IntakeSlidesInCommand(intakeSubsystem, transferSubsystem).withTimeout(250),
                           // new ParallelCommandGroup(

                            //     new IntakeSlidesInCommand(intakeSubsystem, transferSubsystem).withTimeout(250),
                            //     new ActionCommand(secondSample, new ArraySet<>()),
                           // ),


                          /*  new SequentialCommandGroup(

                                    new IntakeSlidesOutCommand(intakeSubsystem),
                                    new IntakePivotDownCommand(intakeSubsystem, robotState)
                            ),
                            new ParallelCommandGroup(

                                    //new SequentialCommandGroup(
                                    //        new WaitCommand(100), //give time for the movement to start
                                     new ColourAwareIntakeCommand(intakeSubsystem).withTimeout(2000)

                                    //),
                                    ,new ActionCommand(secondSampleMoveIn, new ArraySet<>())
                            ),

                            new ConditionalCommand(
                                    new AutoSpecimenCommandGroup(intakeSubsystem, transferSubsystem, robotState),
                                    new SequentialCommandGroup(
                                            new IntakeOffCommand(intakeSubsystem), //just retracting with no sample - do nothing
                                            new IntakeSlidesInCommand(intakeSubsystem, transferSubsystem).withTimeout(500)
                                    ),
                                    () -> true// intakeSubsystem.hasItemInIntake()
                            ),

                            new ActionCommand(secondSampleHuman , new ArraySet<>()),
                            new ParallelCommandGroup(
                                    new ActionCommand(firstHumanMove, new ArraySet<>()),
                                    new IntakeSlidesOutHalfCommand(intakeSubsystem)
                            ),
                            new OuttakeOnCommand(intakeSubsystem),
                            new WaitCommand(500),
                            new IntakeOffCommand(intakeSubsystem),
                            new ParallelCommandGroup(

                                    new IntakeSlidesInCommand(intakeSubsystem, transferSubsystem).withTimeout(250)

                            ),*/

                            //now ready to move for drop off to human

                            new ActionCommand(firstInteractionWait, new ArraySet<>()),

                            new WaitCommand(1500), // let the human line it up

                            new SequentialCommandGroup(
                                    new IntakeSlidesOutCommand(intakeSubsystem),
                                    new IntakePivotDownCommand(intakeSubsystem, robotState)
                            ),
                            new ParallelCommandGroup(
                                    new ActionCommand(firstInteractionMoveIn, new ArraySet<>()),
                                    new SequentialCommandGroup(
                                            new WaitCommand(600), //
                                            new ColourAwareIntakeCommand(intakeSubsystem).withTimeout(2500)
                                    )

                            ),
                            new ConditionalCommand(
                                    new IntakeCommandGroup(intakeSubsystem, transferSubsystem, robotState),
                                    new SequentialCommandGroup(
                                            new IntakeOffCommand(intakeSubsystem), //just retracting with no sample - do nothing
                                            new IntakeSlidesInCommand(intakeSubsystem, transferSubsystem).withTimeout(500)
                                    ),
                                    () -> true// intakeSubsystem.hasItemInIntake()
                            ),


                            new ActionCommand(humanSlamOne, new ArraySet<>()),
                            new WaitCommand(100),
                            new TransferFlipCommand(transferSubsystem),
                            new WaitCommand(500),
                            new SequentialCommandGroup(

                                    new OpenGripplerCommand(transferSubsystem),
                                    new TransferStowCommand(transferSubsystem),
                                    new IntakePivotUpCommand(intakeSubsystem, robotState)

                            ),

                            new ActionCommand(humanInteractionTwo, new ArraySet<>()),

                            new WaitCommand(1500),

                            new SequentialCommandGroup(
                                    new IntakeSlidesOutCommand(intakeSubsystem),
                                    new IntakePivotDownCommand(intakeSubsystem, robotState)
                            ),
                            new ParallelCommandGroup(
                                    new ActionCommand(humanInteractionTwoMoveIn, new ArraySet<>()),
                                    new SequentialCommandGroup(
                                            new WaitCommand(300), //give time for the movement to start
                                            new ColourAwareIntakeCommand(intakeSubsystem).withTimeout(2000)
                                    )

                            ),
                            new ConditionalCommand(
                                    new IntakeCommandGroup(intakeSubsystem, transferSubsystem, robotState),
                                    new SequentialCommandGroup(
                                            new IntakeOffCommand(intakeSubsystem), //just retracting with no sample - do nothing
                                            new IntakeSlidesInCommand(intakeSubsystem, transferSubsystem).withTimeout(500)
                                    ),
                                    () -> true// intakeSubsystem.hasItemInIntake()
                            ),

                            new ActionCommand(humanSlamTwo, new ArraySet<>()),
                            new WaitCommand(100),
                            new TransferFlipCommand(transferSubsystem),
                            new WaitCommand(500),
                            new SequentialCommandGroup(

                                    new OpenGripplerCommand(transferSubsystem),
                                    new TransferStowCommand(transferSubsystem),
                                    new IntakePivotUpCommand(intakeSubsystem, robotState)

                            ),

                            new ActionCommand(humanInteractionThree, new ArraySet<>()),

                            new WaitCommand(500),



                            new InstantCommand(()->{

                                //TODO: update with the final pose of the robot
                                PoseStorage.currentPose = new Pose2d(0, 0, Math.toRadians(-270));

                            })


                    )
                )
        );

    }

}
