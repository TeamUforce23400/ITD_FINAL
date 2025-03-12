package org.firstinspires.ftc.teamcode.opmodes.auto;
import androidx.collection.ArraySet;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.commands.ActionCommand;
import org.firstinspires.ftc.teamcode.commands.AscentOpenHooksCommand;
import org.firstinspires.ftc.teamcode.commands.IntakePivotUpCommand;
import org.firstinspires.ftc.teamcode.commands.OpenGripplerCommand;
import org.firstinspires.ftc.teamcode.commands.SlidesStowCommand;
import org.firstinspires.ftc.teamcode.commands.TransferFlipCommand;
import org.firstinspires.ftc.teamcode.commands.TransferStowCommand;
import org.firstinspires.ftc.teamcode.commands.groups.DeliveryResetCommandGroup;
import org.firstinspires.ftc.teamcode.subsystems.AscentSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotStateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlidesSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;
import org.firstinspires.ftc.teamcode.utils.OTOSDrive;
import org.firstinspires.ftc.teamcode.utils.PoseStorage;

@Autonomous(name = "Red Co Op", group = "Autonomous")
@Disabled
public class RedCoOpAuto extends CommandOpMode {

    Action dropOffPreload;
    Action firstSample;
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
                new Pose2d(-14.8, -64, Math.toRadians(-90)));

        //pose to the submersible wall
        Pose2d dropOffPose = new Pose2d(-3.4, -32.5, Math.toRadians(-90));

        dropOffPreload = drive.actionBuilder(drive.pose)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(dropOffPose, Math.toRadians(90))
                .build();

        //pose to the first sample
        Pose2d firstSamplePose = new Pose2d(-37,-59,Math.toRadians(-240));

        firstSample = drive.actionBuilder(dropOffPose)
                .setTangent(Math.toRadians(-131))
                .splineToLinearHeading(firstSamplePose,Math.toRadians(-131))
                .build();

        park = drive.actionBuilder(firstSamplePose)
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
                            new WaitCommand(400),
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
                            new InstantCommand(()->{

                                PoseStorage.currentPose = new Pose2d(0, 0, Math.toRadians(-270));

                            }),
                            new ParallelCommandGroup(

                                    new ActionCommand(park, new ArraySet<>()),
                                    new DeliveryResetCommandGroup(intakeSubsystem,transferSubsystem,slidesSubsystem, robotState)
                            ),
                            new AscentOpenHooksCommand(ascentSubsystem)



                    )
                )
        );

    }

}
