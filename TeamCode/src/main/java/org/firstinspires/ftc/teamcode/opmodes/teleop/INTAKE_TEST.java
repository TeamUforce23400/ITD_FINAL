package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.SlidesDownJoyCommand;
import org.firstinspires.ftc.teamcode.commands.SlidesHighChamberCommand;
import org.firstinspires.ftc.teamcode.commands.SlidesSpecDrop;
import org.firstinspires.ftc.teamcode.commands.SlidesUpJoyCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlidesSubsystem;
import org.firstinspires.ftc.teamcode.utils.PoseStorage;

@TeleOp(name = "Intake Test")
public class INTAKE_TEST extends CommandOpMode {

    private DriveSubsystem m_drive;

//    private IntakeSubsystem intakeSubsystem;
//
//    private RobotStateSubsystem robotState;
//    private TransferSubsystem transferSubsystem;
    private SlidesSubsystem slidesSubsystem;
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


        m_driveDriver = new GamepadEx(gamepad1);
        m_driveOperator = new GamepadEx(gamepad2);

//        intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetry);
//        robotState = new RobotStateSubsystem();
//
//        //SETUP the starting COLOUR:
//
//
//        transferSubsystem = new TransferSubsystem(hardwareMap);
        slidesSubsystem = new SlidesSubsystem(hardwareMap, telemetry);

//
//        ascentSubsystem = new AscentSubsystem(hardwareMap);


//        intakeSubsystem.setDesiredColour(IntakeSubsystem.SampleColour.BLUE_OR_NEUTRAL);




        //reset the pose from the auto - only if added.

        if(PoseStorage.currentPose != null){

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


//        m_driveDriver.getGamepadButton(GamepadKeys.Button.B).whenActive(
//                new SequentialCommandGroup(
////                        new OpenGripplerCommand(transferSubsystem),
//                        new InstantCommand(()-> {
//                            driveSpeed = 0.4;
//                        }),
//                        new PoopChuteOpenCommand(intakeSubsystem),
//                        new IntakePivotDownCommand(intakeSubsystem, robotState),
//                        new ColourAwareIntakeCommand(intakeSubsystem)
//                )
//        ).whenInactive(
//                //retract and stage if we have the sample
//                new SequentialCommandGroup(
//                        new IntakeOffCommand(intakeSubsystem),
//                        new InstantCommand(()-> {
//                            driveSpeed = 1;
//                        }),
//                        new ConditionalCommand(
//                                new IntakeCommandGroup(intakeSubsystem, transferSubsystem, robotState), // ready to transfer
//                                new InstantCommand(), // do nothing, might want to intake again
//                                ()-> intakeSubsystem.hasItemInIntake()
//                        )
//                )
//
//        );
//
//        new Trigger(new BooleanSupplier() {
//            @Override
//            public boolean getAsBoolean() {
//                return m_driveDriver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5;
//            }
//        }).whileActiveContinuous(
//                new ConditionalCommand(
//                        new SequentialCommandGroup(
//                                new IntakePivotUpCommand(intakeSubsystem, robotState),
//                                new InstantCommand(intakeSubsystem::IncrSlides),
//                                new InstantCommand(()-> {
//                                    driveSpeed = 0.7;
//                                })
//                        ),
//                        new InstantCommand(intakeSubsystem::IncrSlides),
//                        () -> robotState.pivotPosition == RobotStateSubsystem.PivotState.LOW
//                )
//        );
//
////        m_driveDriver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenActive(
////                new SequentialCommandGroup(
////                        new IntakeSlidesInCommand(intakeSubsystem, transferSubsystem)
////                )
////        );
//
//
//
//



//        m_driveDriver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenActive(
//                new SequentialCommandGroup(
//                        new IntakeSlidesInCommand(intakeSubsystem, transferSubsystem)
//                )
//        );



        m_driveOperator.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new SlidesHighChamberCommand(slidesSubsystem)
        );

        m_driveOperator.getGamepadButton(GamepadKeys.Button.X).whenPressed(

                new SlidesSpecDrop(slidesSubsystem)
        );

        m_driveOperator.getGamepadButton(GamepadKeys.Button.DPAD_UP).whileActiveContinuous(
                new SlidesUpJoyCommand(slidesSubsystem)
        );

        m_driveOperator.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whileActiveContinuous(
                new SlidesDownJoyCommand(slidesSubsystem)
        );
    }
}


