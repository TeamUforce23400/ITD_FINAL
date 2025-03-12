package org.firstinspires.ftc.teamcode.opmodes.auto;
import androidx.collection.ArraySet;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
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

@Autonomous(name = "BasketAuto Ex", group = "Autonomous")
@Disabled
public class BasketAutoEx extends CommandOpMode {

    Action dropOffPreload;
    Action firstSample;

    Action firstSampleSlowMoveIn;
    Action deliverFirstSample;
    Action deliverFirstSampleMoveIn;
    Action secondSample;
    Action secondSampleSlowMoveIn;
    Action deliverSecondSample;
    Action deliverSecondSampleMoveIn;
    Action thirdSample;
    Action thirdSampleSlowMoveIn;
    Action deliverThirdSample;

    Action deliverThirdSampleMoveIn;
    Action park;


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
        OTOSDrive drive = new OTOSDrive(hardwareMap,
                new Pose2d(-14.8, -64, Math.toRadians(-90)));

        //pose to the submersible wall
        Pose2d dropOffPose = new Pose2d(-3.4, -32.5, Math.toRadians(-90));

        dropOffPreload = drive.actionBuilder(drive.pose)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(dropOffPose, Math.toRadians(90))
                .build();

        //pose to the first sample
        Pose2d firstSamplePose = new Pose2d(-35 + X_OFFSET,-60 + Y_OFFSET,Math.toRadians(-260));

        firstSample = drive.actionBuilder(dropOffPose)
                .setTangent(Math.toRadians(-260))
                .splineToLinearHeading(firstSamplePose,Math.toRadians(-260))
                .build();

        //pose after the slow move in
        Pose2d slowMoveForward = new Pose2d(-35 + X_OFFSET, -54 + Y_OFFSET, Math.toRadians(-260));

        firstSampleSlowMoveIn = drive.actionBuilder(firstSamplePose)
                .setTangent(Math.toRadians(-260))
                .splineToLinearHeading(slowMoveForward,Math.toRadians(-260), new TranslationalVelConstraint(4))
                 .build();

        Pose2d deliverPose = new Pose2d(-43 + X_OFFSET,-58 + Y_OFFSET,Math.toRadians(-315));

        deliverFirstSample = drive.actionBuilder(slowMoveForward)
                //pick up the first sample
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(deliverPose,Math.toRadians(180))
                .build();

        Pose2d deliverFirstSampleMoveInPose = new Pose2d(-48 + X_OFFSET,-61 + Y_OFFSET, Math.toRadians(-315));

        deliverFirstSampleMoveIn = drive.actionBuilder(deliverPose)
                .splineToLinearHeading(deliverFirstSampleMoveInPose, Math.toRadians(-90))
                .build();

        //Y -64 is on the wall

        Pose2d secondSamplePose = new Pose2d(-46 + X_OFFSET,-58 + Y_OFFSET,Math.toRadians(100));

        secondSample = drive.actionBuilder(deliverPose)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(secondSamplePose,Math.toRadians(90))
                .build();

        Pose2d secondSampleMoveInPose = new Pose2d(-46 + X_OFFSET,-50 + Y_OFFSET,Math.toRadians(100));


        secondSampleSlowMoveIn = drive.actionBuilder(secondSamplePose)
                .splineToLinearHeading(secondSampleMoveInPose, Math.toRadians(90), new TranslationalVelConstraint(5))
                .build();

        Pose2d deliverSecondSamplePose = new Pose2d(-50 + X_OFFSET,-55 + Y_OFFSET,Math.toRadians(-315));

        deliverSecondSample = drive.actionBuilder(secondSampleMoveInPose)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(deliverSecondSamplePose,Math.toRadians(-90))
                .build();

        deliverSecondSampleMoveIn = drive.actionBuilder(deliverSecondSamplePose)
                .splineToLinearHeading(deliverFirstSampleMoveInPose, Math.toRadians(-90))
                .build();

        Pose2d thirdSamplePose = new Pose2d(-46 + X_OFFSET,-59 + Y_OFFSET,Math.toRadians(-242));

        thirdSample = drive.actionBuilder(deliverFirstSampleMoveInPose)
                .setTangent(Math.toRadians(103))
                .splineToLinearHeading(thirdSamplePose,Math.toRadians(103))
                .build();

        Pose2d thirdSampleMoveInPose = new Pose2d(-49 + X_OFFSET,-43 + Y_OFFSET,Math.toRadians(-242));

        thirdSampleSlowMoveIn = drive.actionBuilder(thirdSamplePose)
                .splineToLinearHeading(thirdSampleMoveInPose, Math.toRadians(90), new TranslationalVelConstraint(6))
                .build();

        Pose2d deliverThirdMovePose = new Pose2d(-50 + X_OFFSET,-55 + Y_OFFSET,Math.toRadians(-315));

        deliverThirdSample = drive.actionBuilder(new Pose2d(-46 + X_OFFSET,-49 + Y_OFFSET,Math.toRadians(-242)))
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(deliverThirdMovePose , Math.toRadians(-315))
                .build();

        Pose2d deliverThirdSampleMoveInPose = new Pose2d(-48 + X_OFFSET,-61 + Y_OFFSET, Math.toRadians(-315));

        deliverThirdSampleMoveIn = drive.actionBuilder(deliverThirdMovePose)
                .splineToLinearHeading(deliverThirdSampleMoveInPose, Math.toRadians(-90))
                .build();

        park = drive.actionBuilder(new Pose2d(-50,-55,Math.toRadians(-315)))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-10,-10,Math.toRadians(180)),Math.toRadians(0))
                .build();

        intakeSubsystem.setDesiredColour(IntakeSubsystem.SampleColour.NEUTRAL);
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

                            new ActionCommand(firstSample, new ArraySet<>()),

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
                                    new IntakeCommandGroup(intakeSubsystem, transferSubsystem, robotState),
                                    new SequentialCommandGroup(
                                            new IntakeOffCommand(intakeSubsystem), //just retracting with no sample - do nothing
                                            new IntakeSlidesInCommand(intakeSubsystem, transferSubsystem).withTimeout(500)
                                    ),
                                    () -> true// intakeSubsystem.hasItemInIntake()
                            ),


                            new SequentialCommandGroup(
                                    new CloseGripplerCommand(transferSubsystem),
                                    new WaitCommand(200),
                                    new ConditionalCommand(
                                            // do the drop off if we have the sample
                                             new SequentialCommandGroup(
                                                    new ParallelCommandGroup(
                                                         //new SequentialCommandGroup(
                                                            new ActionCommand(deliverFirstSample, new ArraySet<>()),
                                                            new DeliveryCommandGroup(intakeSubsystem, transferSubsystem, slidesSubsystem, robotState )
                                                         //)
                                                    ),
                                                     new ActionCommand(deliverFirstSampleMoveIn, new ArraySet<>()),
                                                     new OpenGripplerCommand(transferSubsystem),
                                                     new WaitCommand(250)

                                             ),
                                             //don't do the drop off - we don't have the sample
                                             new SequentialCommandGroup(
                                                     /*new InstantCommand(() ->{
                                                        telemetry.addData("No Item", "No sample found");
                                                        telemetry.update();
                                                     }),*/
                                                     new IntakeOffCommand(intakeSubsystem),
                                                     new IntakeSlidesInCommand(intakeSubsystem, transferSubsystem).withTimeout(500)
                                             ),
                                            () -> true//intakeSubsystem.hasItemInIntake()
                                            //TODO: The hasItemInIntake needs some work - not consistent
                                    )

                            ),

                            new ParallelCommandGroup(

                                    new ActionCommand(secondSample, new ArraySet<>()),
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
                                        new ActionCommand(secondSampleSlowMoveIn, new ArraySet<>())
                                    )

                            ),

                            new ConditionalCommand(
                                    new IntakeCommandGroup(intakeSubsystem, transferSubsystem, robotState),
                                    new SequentialCommandGroup( //don't have anything in the intake
                                            new IntakeOffCommand(intakeSubsystem),
                                            new IntakeSlidesInCommand(intakeSubsystem, transferSubsystem).withTimeout(500)
                                    ),
                                    () -> true// intakeSubsystem.hasItemInIntake()
                            ),


                            new SequentialCommandGroup(
                                    new CloseGripplerCommand(transferSubsystem),
                                    new ConditionalCommand(
                                            // do the drop off if we have the sample
                                            new SequentialCommandGroup(
                                                    new ParallelCommandGroup(
                                                            new ActionCommand(deliverSecondSample, new ArraySet<>()),
                                                            new DeliveryCommandGroup(intakeSubsystem, transferSubsystem, slidesSubsystem, robotState )
                                                    ),
                                                    new ActionCommand(deliverSecondSampleMoveIn, new ArraySet<>()),
                                                    new OpenGripplerCommand(transferSubsystem),
                                                    new WaitCommand(250)
                                            ),
                                            //don't do the drop off - we don't have the sample
                                            new SequentialCommandGroup(
                                                    new IntakeOffCommand(intakeSubsystem),
                                                    new IntakeSlidesInCommand(intakeSubsystem, transferSubsystem).withTimeout(500)
                                            ),
                                            () -> true // intakeSubsystem.hasItemInIntake()
                                    )
                            ),


                            new ParallelCommandGroup(

                                    new ActionCommand(thirdSample, new ArraySet<>()),
                                    new DeliveryResetCommandGroup(intakeSubsystem,transferSubsystem,slidesSubsystem, robotState)
                            ),
                            //third sample


                            new ParallelCommandGroup(
                                    new SequentialCommandGroup(
                                            new IntakeSlidesOutCommand(intakeSubsystem),
                                            new IntakePivotDownCommand(intakeSubsystem, robotState),
                                            new ColourAwareIntakeCommand(intakeSubsystem).withTimeout(2000)
                                    ),
                                    //new SequentialCommandGroup(
                                    //        new WaitCommand(300), //time to settle in
                                            new ActionCommand(thirdSampleSlowMoveIn, new ArraySet<>())
                                    //)
                            ),

                            new ConditionalCommand(
                                    new IntakeCommandGroup(intakeSubsystem, transferSubsystem, robotState),
                                    new SequentialCommandGroup( //don't have anything in the intake
                                            new IntakeOffCommand(intakeSubsystem),
                                            new IntakeSlidesInCommand(intakeSubsystem, transferSubsystem).withTimeout(500)
                                    ),
                                    () -> true //intakeSubsystem.hasItemInIntake()
                            ),

                            new SequentialCommandGroup(
                                    new ParallelCommandGroup(
                                            new ActionCommand(deliverThirdSample, new ArraySet<>()),
                                            new DeliveryCommandGroup(intakeSubsystem, transferSubsystem, slidesSubsystem, robotState )
                                    ),
                                    new ActionCommand(deliverThirdSampleMoveIn, new ArraySet<>()),
                                    new OpenGripplerCommand(transferSubsystem),
                                    new WaitCommand(250)
                            ),



                            new ParallelCommandGroup(
                                    new ActionCommand(park, new ArraySet<>()),
                                    new DeliveryResetCommandGroup(intakeSubsystem,transferSubsystem,slidesSubsystem, robotState),
                                    new AscentOpenHooksCommand(ascentSubsystem)
                            ),

                            new InstantCommand(()->{

                                PoseStorage.currentPose = new Pose2d(0, 0, Math.toRadians(-270));

                            })

                          //  new ActionCommand(park, new ArraySet<>())

                    )
                )
        );

    }

}
