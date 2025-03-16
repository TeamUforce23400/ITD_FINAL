package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.AscentCloseHooksCommand;
import org.firstinspires.ftc.teamcode.commands.AscentLowRungCommand;
import org.firstinspires.ftc.teamcode.commands.AscentOpenHooksCommand;
import org.firstinspires.ftc.teamcode.commands.AscentStowCommand;
import org.firstinspires.ftc.teamcode.commands.CloseGripplerCommand;
import org.firstinspires.ftc.teamcode.commands.ColourAwareIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.DefaultDrive;
import org.firstinspires.ftc.teamcode.commands.DesiredColourBlueCommand;
import org.firstinspires.ftc.teamcode.commands.DesiredColourNeutralCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeOffCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeOnCommand;
import org.firstinspires.ftc.teamcode.commands.IntakePivotDownCommand;
import org.firstinspires.ftc.teamcode.commands.IntakePivotUpCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeSlidesInCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeSlidesOutCommand;
import org.firstinspires.ftc.teamcode.commands.OpenGripplerCommand;
import org.firstinspires.ftc.teamcode.commands.OuttakeOnCommand;
import org.firstinspires.ftc.teamcode.commands.PoopChuteCloseCommand;
import org.firstinspires.ftc.teamcode.commands.PoopChuteOpenCommand;
import org.firstinspires.ftc.teamcode.commands.SlidesHighBasketCommand;
import org.firstinspires.ftc.teamcode.commands.SlidesLowBasketCommand;
import org.firstinspires.ftc.teamcode.commands.SlidesStowCommand;
import org.firstinspires.ftc.teamcode.commands.TransferFlipCommand;
import org.firstinspires.ftc.teamcode.commands.TransferStowCommand;
import org.firstinspires.ftc.teamcode.commands.groups.IntakeCommandGroup;
import org.firstinspires.ftc.teamcode.subsystems.AscentSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotStateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlidesSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;
import org.firstinspires.ftc.teamcode.utils.PoseStorage;

import java.util.function.BooleanSupplier;
@TeleOp(name = "Intake Test")
public class INTAKE_TEST extends CommandOpMode {

    private DriveSubsystem m_drive;
    private DefaultDrive m_driveCommand;
    private IntakeSubsystem intakeSubsystem;

    private RobotStateSubsystem robotState;
    private TransferSubsystem transferSubsystem;
//    private SlidesSubsystem slidesSubsystem;
//
//    private AscentSubsystem ascentSubsystem;
//
//    private RobotStateSubsystem robotState;
    private GamepadEx m_driveDriver;
    private GamepadEx m_driveOperator;

    private double driveSpeed = 1;


    @Override
    public void initialize() {
//        robotState = new RobotStateSubsystem();
        m_drive = new DriveSubsystem(hardwareMap, telemetry);

        m_driveDriver = new GamepadEx(gamepad1);
        m_driveOperator = new GamepadEx(gamepad2);

        intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetry);
        robotState = new RobotStateSubsystem();

        //SETUP the starting COLOUR:


        transferSubsystem = new TransferSubsystem(hardwareMap);
//
//        slidesSubsystem = new SlidesSubsystem(hardwareMap);
//
//        ascentSubsystem = new AscentSubsystem(hardwareMap);


        intakeSubsystem.setDesiredColour(IntakeSubsystem.SampleColour.BLUE_OR_NEUTRAL);
        m_driveCommand = new DefaultDrive(m_drive, () -> m_driveDriver.getLeftX(),  () -> m_driveDriver.getLeftY(), () -> m_driveDriver.getRightX() * 0.5 , ()-> driveSpeed);

        register(m_drive);
        m_drive.setDefaultCommand(m_driveCommand);

        //reset the pose from the auto - only if added.

        if(PoseStorage.currentPose != null){
            m_drive.setPose(PoseStorage.currentPose);
            //we've used it up now, clear it
            PoseStorage.currentPose = null;
        }

        //get rid of this when not needed
        /*schedule(new RunCommand(() -> {
                telemetry.addData("Slides:", slidesSubsystem.getCurrentSlidePos());
                telemetry.addData("Magnet", transferSubsystem.IsTransferClosed());
                telemetry.addData("ML", ascentSubsystem.getLeftMotorPos());
                telemetry.addData("MR", ascentSubsystem.getRightMotorPos());
                telemetry.update();
        }
        ));*/


        //Intake
//        new Trigger(new BooleanSupplier() {
//            @Override
//            public boolean getAsBoolean() {
//                return m_driveDriver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5;
//            }
//        }).whenActive(
//                new SequentialCommandGroup(
//
//                        //give the speed back
//                        new InstantCommand(()-> {
//                            driveSpeed = 1;
//                        }),
//                        //make sure we lift the pivot if it is down, it could hit something
//                        new ConditionalCommand(
//                                new SequentialCommandGroup(
//                                        new IntakePivotUpCommand(intakeSubsystem,robotState),
//                                        new WaitCommand(300),
//                                        new IntakeSlidesInCommand(intakeSubsystem,transferSubsystem).withTimeout(500)
//                                ),
//                                new IntakeSlidesInCommand(intakeSubsystem, transferSubsystem).withTimeout(500),
//                                () -> robotState.pivotPosition == RobotStateSubsystem.PivotState.LOW
//                        ),
//
//                        new ConditionalCommand(
//                                new IntakeCommandGroup(intakeSubsystem, transferSubsystem, robotState),
//                                new InstantCommand(), //just retracting with no sample - do nothing
//                                () -> intakeSubsystem.hasItemInIntake()
//                        ),
//                        new InstantCommand(()-> {
//                            driveSpeed = 1;
//                        })
//                )
//
//        );


        //intaking


        m_driveDriver.getGamepadButton(GamepadKeys.Button.B).whenActive(
                new SequentialCommandGroup(
//                        new OpenGripplerCommand(transferSubsystem),
                        new InstantCommand(()-> {
                            driveSpeed = 0.4;
                        }),
                        new PoopChuteOpenCommand(intakeSubsystem),
                        new IntakePivotDownCommand(intakeSubsystem, robotState),
                        new ColourAwareIntakeCommand(intakeSubsystem)
                )
        ).whenInactive(
                //retract and stage if we have the sample
                new SequentialCommandGroup(
                        new IntakeOffCommand(intakeSubsystem),
                        new InstantCommand(()-> {
                            driveSpeed = 1;
                        }),
                        new ConditionalCommand(
                                new IntakeCommandGroup(intakeSubsystem, transferSubsystem, robotState), // ready to transfer
                                new InstantCommand(), // do nothing, might want to intake again
                                ()-> intakeSubsystem.hasItemInIntake()
                        )
                )

        );

        new Trigger(new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return m_driveDriver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5;
            }
        }).whileActiveContinuous(
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new IntakePivotUpCommand(intakeSubsystem, robotState),
                                new InstantCommand(intakeSubsystem::IncrSlides),
                                new InstantCommand(()-> {
                                    driveSpeed = 0.7;
                                })
                        ),
                        new InstantCommand(intakeSubsystem::IncrSlides),
                        () -> robotState.pivotPosition == RobotStateSubsystem.PivotState.LOW
                )
        );

//        m_driveDriver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenActive(
//                new SequentialCommandGroup(
//                        new IntakeSlidesInCommand(intakeSubsystem, transferSubsystem)
//                )
//        );






        m_driveOperator.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new OpenGripplerCommand(transferSubsystem)
        );

        m_driveOperator.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new CloseGripplerCommand(transferSubsystem)
        );
    }
}


