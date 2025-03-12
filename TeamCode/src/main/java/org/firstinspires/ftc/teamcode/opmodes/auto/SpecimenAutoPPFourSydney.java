package org.firstinspires.ftc.teamcode.opmodes.auto;
import androidx.collection.ArraySet;

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

import org.firstinspires.ftc.teamcode.commands.ActionCommand;
import org.firstinspires.ftc.teamcode.commands.ColourAwareIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeOffCommand;
import org.firstinspires.ftc.teamcode.commands.IntakePivotDownCommand;
import org.firstinspires.ftc.teamcode.commands.IntakePivotUpCommand;
import org.firstinspires.ftc.teamcode.commands.IntakePoopChuteOpenCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeSlidesOutCommand;
import org.firstinspires.ftc.teamcode.commands.OpenGripplerCommand;
import org.firstinspires.ftc.teamcode.commands.OuttakeOnCommand;
import org.firstinspires.ftc.teamcode.commands.SlidesStowCommand;
import org.firstinspires.ftc.teamcode.commands.TransferFlipCommand;
import org.firstinspires.ftc.teamcode.commands.TransferStowCommand;
import org.firstinspires.ftc.teamcode.commands.groups.AutoIntakeCommandGroup;
import org.firstinspires.ftc.teamcode.subsystems.AscentSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotStateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlidesSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;
import org.firstinspires.ftc.teamcode.utils.PinpointDrive;
import org.firstinspires.ftc.teamcode.utils.PoseStorage;

@Autonomous(name = "SPECIMEN | FOUR | PARK", group = "Autonomous")
//delivers a specimen, grabs two samples, delivers three more specimens
public class SpecimenAutoPPFourSydney extends CommandOpMode {

    TrajectoryActionBuilder dropOffPreload;
    TrajectoryActionBuilder firstSample;
    TrajectoryActionBuilder firstSampleSlow;
    TrajectoryActionBuilder firstSampleDrop;
    TrajectoryActionBuilder secondSample;
    TrajectoryActionBuilder secondSampleSlow;
    TrajectoryActionBuilder secondSampleDrop;
    TrajectoryActionBuilder thirdSample;
    TrajectoryActionBuilder thirdSampleDrop;
    TrajectoryActionBuilder firstSpecimenPickup;
    TrajectoryActionBuilder tryAgainPickup;
    TrajectoryActionBuilder firstSpecimenDrop;
    TrajectoryActionBuilder secondSpecimenPickup;
    TrajectoryActionBuilder secondSpecimenDrop;
    TrajectoryActionBuilder thirdSpecimenPickup;

    TrajectoryActionBuilder thirdSpecimenDrop;

    TrajectoryActionBuilder park;


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
                new Pose2d(3.5, -64, Math.toRadians(-90)));

        //pose to the submersible wall
        Pose2d dropOffPose = new Pose2d(-2, -31.5, Math.toRadians(-90));

        dropOffPreload = drive.actionBuilder(drive.pose)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(dropOffPose, Math.toRadians(90))
                .endTrajectory();

        Pose2d firstSamplePose = new Pose2d(23,-56,Math.toRadians(58));

        firstSample = dropOffPreload.fresh()
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(firstSamplePose,Math.toRadians(0))
                .endTrajectory();

        Vector2d firstSampleSlowPose = new Vector2d(25,-54);

        firstSampleSlow = firstSample.fresh()
                .strafeTo(firstSampleSlowPose, new TranslationalVelConstraint(70))
                .endTrajectory();

        firstSampleDrop = firstSampleSlow.fresh()
                .turn(Math.toRadians(-90))
                .endTrajectory();

        Pose2d secondSamplePose = new Pose2d(32,-58,Math.toRadians(58));

        secondSample = firstSampleDrop.fresh()
                 .setTangent(Math.toRadians(58))
                 .splineToLinearHeading(secondSamplePose, Math.toRadians(58))
                 .endTrajectory();

        Vector2d secondSampleSlowPose = new Vector2d(35,-54);

        secondSampleSlow = secondSample.fresh()
                .strafeTo(secondSampleSlowPose, new TranslationalVelConstraint(60))
                .endTrajectory();

        secondSampleDrop = secondSampleSlow.fresh()
                .turn(Math.toRadians(-90))
                .endTrajectory();

        //first specimen
        Pose2d firstSpecimenPickupPose = new Pose2d(11,-45,Math.toRadians(-45));
        Vector2d firstSpecimenPickupPoseSlow = new Vector2d(15, -49);

        firstSpecimenPickup = secondSampleDrop.fresh()
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(firstSpecimenPickupPose, Math.toRadians(-45))
                .strafeTo(firstSpecimenPickupPoseSlow, new TranslationalVelConstraint(10))
                .endTrajectory();

        Pose2d firstSpecimenDropPose = new Pose2d(-3.5, -32.5, Math.toRadians(-90));

        firstSpecimenDrop = firstSpecimenPickup.fresh()
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(firstSpecimenDropPose, Math.toRadians(90))
                .endTrajectory();

        Vector2d tryAgainPose = new Vector2d(42,-35);

        tryAgainPickup = firstSpecimenPickup.fresh()
                .strafeTo(tryAgainPose)
                .endTrajectory();

        //second specimen
        Pose2d secondSpecimenPickupPose = new Pose2d(11,-45,Math.toRadians(-45));
        Vector2d secondSpecimenPickupPoseSlow = new Vector2d(15, -49);


        secondSpecimenPickup = firstSpecimenDrop.fresh()
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(secondSpecimenPickupPose, Math.toRadians(-45))
                .strafeTo(secondSpecimenPickupPoseSlow, new TranslationalVelConstraint(10))
                .endTrajectory();

        Pose2d secondSpecimenDropPose = new Pose2d(-5, -32.5, Math.toRadians(-90));

        secondSpecimenDrop = secondSpecimenPickup.fresh()
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(secondSpecimenDropPose, Math.toRadians(90))
                .endTrajectory();

        //third specimen
        Pose2d thirdSpecimenPickupPose = new Pose2d(11,-44,Math.toRadians(-45));
        Vector2d thirdSpecimenPickupPoseSlow = new Vector2d(15, -49);


        thirdSpecimenPickup = secondSpecimenDrop.fresh()
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(thirdSpecimenPickupPose, Math.toRadians(-45))
                .strafeTo(thirdSpecimenPickupPoseSlow, new TranslationalVelConstraint(10))
                .endTrajectory();

        Pose2d thirdSpecimenDropPose = new Pose2d(-6.5, -32.5, Math.toRadians(-90));

        thirdSpecimenDrop = thirdSpecimenPickup.fresh()
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(thirdSpecimenDropPose, Math.toRadians(90))
                .endTrajectory();

        Pose2d parkPose = new Pose2d(35,-60,Math.toRadians(-90));

        park = thirdSpecimenDrop.fresh()
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(parkPose, Math.toRadians(-90))
                .endTrajectory();

        intakeSubsystem.setDesiredColour(IntakeSubsystem.SampleColour.AUTO_ANY);
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

                            //first sample
                            new ParallelCommandGroup(
                                new ActionCommand(firstSample.build(), new ArraySet<>()),
                                new SequentialCommandGroup(
                                    new WaitCommand(1200),
                                    new IntakePoopChuteOpenCommand(intakeSubsystem),
                                    new IntakeSlidesOutCommand(intakeSubsystem),
                                    new IntakePivotDownCommand(intakeSubsystem, robotState)
                                )
                            ),

                            new ParallelCommandGroup(
                                    new ColourAwareIntakeCommand(intakeSubsystem).withTimeout(400),
                                    new ActionCommand(firstSampleSlow.build(), new ArraySet<>())
                            ),
                            new ActionCommand(firstSampleDrop.build(), new ArraySet<>()),
                            new OuttakeOnCommand(intakeSubsystem),
                            new WaitCommand(500),
                            new IntakeOffCommand(intakeSubsystem),

                            //second sample
                            new ActionCommand(secondSample.build(), new ArraySet<>()),
                            new ParallelCommandGroup(
                                    new ColourAwareIntakeCommand(intakeSubsystem).withTimeout(400),
                                    new ActionCommand(secondSampleSlow.build(), new ArraySet<>())
                            ),
                            new ActionCommand(secondSampleDrop.build(), new ArraySet<>()),
                            new OuttakeOnCommand(intakeSubsystem),
                            new WaitCommand(500),
                            new IntakeOffCommand(intakeSubsystem),

                            /*third sample
                            new ActionCommand(thirdSample.build(), new ArraySet<>()),
                            new IntakeSlidesOutCommand(intakeSubsystem),
                            new ColourAwareIntakeCommand(intakeSubsystem).withTimeout(400),
                            new IntakeSlidesInCommand(intakeSubsystem, transferSubsystem).withTimeout(200),
                            new ActionCommand(thirdSampleDrop.build(), new ArraySet<>()),
                            new IntakeSlidesOutCommand(intakeSubsystem),
                            new WaitCommand(500),
                            new OuttakeOnCommand(intakeSubsystem),
                            new WaitCommand(500),
                            new IntakeOffCommand(intakeSubsystem),*/

                            //first specimen drop
                            new ActionCommand(firstSpecimenPickup.build(), new ArraySet<>()),
                            new ColourAwareIntakeCommand(intakeSubsystem).withTimeout(700),
                            new ParallelCommandGroup(
                                new ActionCommand(firstSpecimenDrop.build(), new ArraySet<>()),
                                new AutoIntakeCommandGroup(intakeSubsystem, transferSubsystem, robotState)
                            ),
                            new TransferFlipCommand(transferSubsystem),
                            new WaitCommand(500),
                            new OpenGripplerCommand(transferSubsystem),
                            new TransferStowCommand(transferSubsystem),
                            new IntakePoopChuteOpenCommand(intakeSubsystem),
                            new IntakeSlidesOutCommand(intakeSubsystem),
                            new IntakePivotDownCommand(intakeSubsystem, robotState),

                            //second specimen drop
                            new ActionCommand(secondSpecimenPickup.build(), new ArraySet<>()),
                            new ColourAwareIntakeCommand(intakeSubsystem).withTimeout(700),
                            new ParallelCommandGroup(
                                    new ActionCommand(secondSpecimenDrop.build(), new ArraySet<>()),
                                    new AutoIntakeCommandGroup(intakeSubsystem, transferSubsystem, robotState)
                            ),
                            new TransferFlipCommand(transferSubsystem),
                            new WaitCommand(500),
                            new OpenGripplerCommand(transferSubsystem),
                            new TransferStowCommand(transferSubsystem),
                            new IntakePoopChuteOpenCommand(intakeSubsystem),
                            new IntakeSlidesOutCommand(intakeSubsystem),
                            new IntakePivotDownCommand(intakeSubsystem, robotState),

                            //third specimen drop
                            new ActionCommand(thirdSpecimenPickup.build(), new ArraySet<>()),
                            new ColourAwareIntakeCommand(intakeSubsystem).withTimeout(700),
                            new ParallelCommandGroup(
                                    new ActionCommand(thirdSpecimenDrop.build(), new ArraySet<>()),
                                    new AutoIntakeCommandGroup(intakeSubsystem, transferSubsystem, robotState)
                            ),
                            new TransferFlipCommand(transferSubsystem),
                            new WaitCommand(500),
                            new OpenGripplerCommand(transferSubsystem),
                            new TransferStowCommand(transferSubsystem),
                            new IntakePoopChuteOpenCommand(intakeSubsystem),

                            new InstantCommand(()->{

                                PoseStorage.currentPose = new Pose2d(0, 0, Math.toRadians(180));

                            }),
                            new ActionCommand(park.build(), new ArraySet<>())


                    )
                )
        );

    }

}
