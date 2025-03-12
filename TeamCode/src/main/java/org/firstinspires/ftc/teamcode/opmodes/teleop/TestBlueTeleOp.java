package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.CloseGripplerCommand;
import org.firstinspires.ftc.teamcode.commands.DefaultDrive;
import org.firstinspires.ftc.teamcode.commands.DesiredColourBlueCommand;
import org.firstinspires.ftc.teamcode.commands.DesiredColourNeutralCommand;
import org.firstinspires.ftc.teamcode.commands.DesiredColourRedCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeOffCommand;
import org.firstinspires.ftc.teamcode.commands.IntakePivotDownCommand;
import org.firstinspires.ftc.teamcode.commands.IntakePivotUpCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeSlidesInCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeSlidesOutCommand;
import org.firstinspires.ftc.teamcode.commands.MiddleGripplerRotationCommand;
import org.firstinspires.ftc.teamcode.commands.OpenGripplerCommand;
import org.firstinspires.ftc.teamcode.commands.OuttakeOnCommand;
import org.firstinspires.ftc.teamcode.commands.PoopChuteCloseCommand;
import org.firstinspires.ftc.teamcode.commands.SlidesHighBasketCommand;
import org.firstinspires.ftc.teamcode.commands.SlidesLowBasketCommand;
import org.firstinspires.ftc.teamcode.commands.SlidesStowCommand;
import org.firstinspires.ftc.teamcode.commands.TeleOpIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.TransferFlipCommand;
import org.firstinspires.ftc.teamcode.commands.TransferStowCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotStateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlidesSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;

import java.util.function.BooleanSupplier;

@TeleOp(name = "Test Blue TeleOp")
@Disabled
public class TestBlueTeleOp  extends CommandOpMode {
// rightn trigger: basket
    // left trigger slides out and intake
    // right bumper speciman chamber
    private DriveSubsystem m_drive;
    private DefaultDrive m_driveCommand;
    private IntakeSubsystem intakeSubsystem;
    private TransferSubsystem transferSubsystem;
    private SlidesSubsystem slidesSubsystem;

    private RobotStateSubsystem robotState;
    private GamepadEx m_driveDriver;
    private GamepadEx m_driveOperator;

    private double driveSpeed = 1;
    @Override

    public void initialize() {
        robotState = new RobotStateSubsystem();
        m_drive = new DriveSubsystem(hardwareMap, telemetry);

        m_driveDriver = new GamepadEx(gamepad1);
        m_driveOperator = new GamepadEx(gamepad2);

        intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetry);
        transferSubsystem = new TransferSubsystem(hardwareMap);
        slidesSubsystem = new SlidesSubsystem(hardwareMap, telemetry);



        m_driveCommand = new DefaultDrive(m_drive, () -> m_driveDriver.getLeftX(),  () -> m_driveDriver.getLeftY(), () -> m_driveDriver.getRightX() * 0.5 , ()-> driveSpeed);

        register(m_drive);
        m_drive.setDefaultCommand(m_driveCommand);


        //Intake
        new Trigger(new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return m_driveDriver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5;
            }
        }).whenActive(
                new SequentialCommandGroup(
                    new OpenGripplerCommand(transferSubsystem),
                        new InstantCommand(()-> {
                            driveSpeed = 0.3;
                        }),
                    new WaitCommand(250),
                    new TeleOpIntakeCommand(intakeSubsystem, robotState)
                )

        ).whenInactive(
                new SequentialCommandGroup(
                    new IntakeOffCommand(intakeSubsystem),
                    new PoopChuteCloseCommand(intakeSubsystem),
                    new ParallelCommandGroup(
                        new IntakePivotUpCommand(intakeSubsystem, robotState),
                        new MiddleGripplerRotationCommand(transferSubsystem)
                    ),
                    new WaitCommand(300), //give the servos time to operate
                    new IntakeSlidesInCommand(intakeSubsystem, transferSubsystem),
                    //new WaitCommand(900), //TODO, wait for now, but we'll look to use a sensor
                    new ConditionalCommand(
                        new WaitCommand(1),
                        new SequentialCommandGroup(
                            new CloseGripplerCommand(transferSubsystem),
                            new IntakePivotDownCommand(intakeSubsystem, robotState)),
                        () -> {return intakeSubsystem.getCurrentIntakeColour() == IntakeSubsystem.SampleColour.NONE;}
                    ),
                    new InstantCommand(()-> {
                        driveSpeed = 1;
                        })
                )
        );

        //High basket sample
        new Trigger(new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return m_driveDriver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5;
            }
        }).whenActive(

            //new ParallelCommandGroup(
               //
                new SequentialCommandGroup(
                        new CloseGripplerCommand(transferSubsystem),
                        new WaitCommand(200),
                        new ParallelCommandGroup(
                            new SlidesHighBasketCommand(slidesSubsystem),
                            new TransferFlipCommand(transferSubsystem)
                        ),
                        new InstantCommand(()-> {
                            driveSpeed = 0.5;
                        })
                )
            ).whenInactive(
                new SequentialCommandGroup(
                    new OpenGripplerCommand(transferSubsystem),
                    new WaitCommand(300),
                    new TransferStowCommand(transferSubsystem),
                    new InstantCommand(()-> {
                            driveSpeed = 1;
                        }),
                        new IntakePivotUpCommand(intakeSubsystem, robotState),
                    new SlidesStowCommand(slidesSubsystem)

                )
        );

        //High chamber specimen
        m_driveDriver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new ParallelCommandGroup(
                       // new SlidesHighChamberCommand(slidesSubsystem),
                        new TransferFlipCommand(transferSubsystem),
                        new InstantCommand(()-> {
                            driveSpeed = 0.5;
                        })
                )).whenInactive(
                new SequentialCommandGroup(
                       // new SlidesHighChamberDeliverCommand(slidesSubsystem),
                        //new WaitCommand(200),
                        new OpenGripplerCommand(transferSubsystem),
                        new TransferStowCommand(transferSubsystem),
                        new InstantCommand(()-> {
                            driveSpeed = 1;
                        }),
                        new IntakePivotUpCommand(intakeSubsystem, robotState),
                        new SlidesStowCommand(slidesSubsystem)
                )
        );

        //Low Basket
        m_driveDriver.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new ParallelCommandGroup(
                        new SlidesLowBasketCommand(slidesSubsystem),
                        new TransferFlipCommand(transferSubsystem),
                        new InstantCommand(()-> {
                            driveSpeed = 0.5;
                        })
                )).whenInactive(
                new SequentialCommandGroup(
                        new OpenGripplerCommand(transferSubsystem),
                        new WaitCommand(350),
                        new TransferStowCommand(transferSubsystem),
                        new IntakePivotUpCommand(intakeSubsystem, robotState),
                        new SlidesStowCommand(slidesSubsystem),
                        new InstantCommand(()-> {
                            driveSpeed = 1;
                        })
                )
        );

        //alex yucky code please delete
        m_driveDriver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
              new SequentialCommandGroup(
                new IntakeSlidesOutCommand(intakeSubsystem),
                new WaitCommand(300),
                new IntakePivotDownCommand(intakeSubsystem, robotState),
                new InstantCommand(()-> {
                  driveSpeed = 0.4;
                })
              )

        );

        m_driveDriver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
              new SequentialCommandGroup(
                      new IntakePivotUpCommand(intakeSubsystem, robotState),
                      new WaitCommand(500),
                      new IntakeSlidesInCommand(intakeSubsystem, transferSubsystem),
                      new InstantCommand(()-> {
                          driveSpeed = 1;
                      })

              )

        );

        m_driveDriver.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
            new OuttakeOnCommand(intakeSubsystem)
        ).whenReleased(
                new IntakeOffCommand(intakeSubsystem)
        );

        m_driveOperator.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new DesiredColourBlueCommand(intakeSubsystem)
        );

        m_driveOperator.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new DesiredColourRedCommand(intakeSubsystem)
        );

        m_driveOperator.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new DesiredColourNeutralCommand(intakeSubsystem)
        );

        m_driveOperator.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new InstantCommand(()-> {
                    driveSpeed = 1;
                })
        );
    }
}


