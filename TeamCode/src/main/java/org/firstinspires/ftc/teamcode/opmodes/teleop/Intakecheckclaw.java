package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.ClawCloseCommand;
import org.firstinspires.ftc.teamcode.commands.ClawLooseCommand;
import org.firstinspires.ftc.teamcode.commands.CloseGripplerCommand;
import org.firstinspires.ftc.teamcode.commands.DefaultDrive;
import org.firstinspires.ftc.teamcode.commands.FireOneShotCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeClawOpenCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeColorNeutralCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeColorRedCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeColorRedOrNeutralCommand;
import org.firstinspires.ftc.teamcode.commands.IntakePivotUpCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeSlidesInCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeSlidesOutCommand;
import org.firstinspires.ftc.teamcode.commands.OpenGripplerCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeResetCommand;
import org.firstinspires.ftc.teamcode.commands.SlidesHighBasketCommand;
import org.firstinspires.ftc.teamcode.commands.SlidesHighChamberCommand;
import org.firstinspires.ftc.teamcode.commands.SlidesStowCommand;
import org.firstinspires.ftc.teamcode.commands.TransferBackwardCommand;
import org.firstinspires.ftc.teamcode.commands.TransferFlipCommand;
import org.firstinspires.ftc.teamcode.commands.TransferSpecimenDropCommand;
import org.firstinspires.ftc.teamcode.commands.TransferStowCommand;
import org.firstinspires.ftc.teamcode.commands.TurretNormalResetCommand;
import org.firstinspires.ftc.teamcode.commands.TurretResetTransferCommand;
import org.firstinspires.ftc.teamcode.commands.TurretSideTransferCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotStateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlidesSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;

@TeleOp(name = "Intake check")
public class Intakecheckclaw extends CommandOpMode {

    private DriveSubsystem m_drive;
    private DefaultDrive m_driveCommand;
    private IntakeSubsystem     intakeSubsystem;
    private TransferSubsystem   transferSubsystem;
    private SlidesSubsystem     slidesSubsystem;
    private RobotStateSubsystem robotState;
    private double driveSpeed = 1.0;

    private GamepadEx driver;
    private GamepadEx operator;


    @Override
    public void initialize() {
        // 1) Gamepads
        driver   = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);


        intakeSubsystem   = new IntakeSubsystem(hardwareMap, telemetry);
        transferSubsystem = new TransferSubsystem(hardwareMap);

        slidesSubsystem = new SlidesSubsystem(hardwareMap, telemetry);
        robotState = new RobotStateSubsystem();
        m_drive = new DriveSubsystem(hardwareMap, telemetry);

        m_driveCommand = new DefaultDrive(m_drive, () -> driver.getLeftX(),  () -> driver.getLeftY(), () -> driver.getRightX() * 0.5 , ()-> driveSpeed);


        register(m_drive);
        m_drive.setDefaultCommand(m_driveCommand);

        //Sample Intake
        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new SequentialCommandGroup(
                        //Reset Transfer
                        new ParallelCommandGroup(
                                new TransferStowCommand(transferSubsystem),
                                new OpenGripplerCommand(transferSubsystem)
                        ),
                        //Reset Intake
                        new IntakeResetCommand(intakeSubsystem),
                        //Auto Aim one shot
                        new FireOneShotCommand(intakeSubsystem),
                        new WaitCommand(500),
                        new ClawCloseCommand(intakeSubsystem),
                        //Retract for Transfer
                        new IntakePivotUpCommand(intakeSubsystem, robotState),
                        new ParallelCommandGroup(
                                new TurretResetTransferCommand(intakeSubsystem),
                                new IntakeSlidesInCommand(intakeSubsystem, transferSubsystem),
                                new WaitCommand(1000),
                                new ClawLooseCommand(intakeSubsystem)
                        ),
                        //Transfer
                        new WaitCommand(200),
                        new CloseGripplerCommand(transferSubsystem),
                        new IntakeClawOpenCommand(intakeSubsystem),
                        new IntakeResetCommand(intakeSubsystem),
                        //Cascades Up & Flip arm to drop
                        new ParallelCommandGroup(
                                new SlidesHighBasketCommand(slidesSubsystem),
                                new TransferFlipCommand(transferSubsystem)
                        )

                )
        );

        //Add Color Choice Buttons
        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
            new IntakeColorRedCommand(intakeSubsystem)
        );

        driver.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new IntakeColorRedOrNeutralCommand(intakeSubsystem)
        );

        driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                new IntakeColorNeutralCommand(intakeSubsystem)
        );


        //Drop, Reset Slides & Stow command add
        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new SequentialCommandGroup(
                        new OpenGripplerCommand(transferSubsystem),
                        new ParallelCommandGroup(
                                new SlidesStowCommand(slidesSubsystem),
                                new TransferStowCommand(transferSubsystem),
                                new IntakeResetCommand(intakeSubsystem)
                        )
                )
        );

        // 6) Button X: reset arm (raise pivots & resume sampling)
        driver.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new IntakeResetCommand(intakeSubsystem));

        //Specimen Pick from Larger side of submersible
        operator.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new SequentialCommandGroup(
                        //Reset Transfer
                        new ParallelCommandGroup(
                                //Add backwards specimen transfer
                                new OpenGripplerCommand(transferSubsystem)
                        ),
                        //Reset Intake
                        new IntakeResetCommand(intakeSubsystem),
                        //Auto Aim one shot
                        new FireOneShotCommand(intakeSubsystem),
                        new WaitCommand(500),
                        new ClawCloseCommand(intakeSubsystem),
                        new IntakePivotUpCommand(intakeSubsystem, robotState),
                        //Retract for Transfer
                        new ParallelCommandGroup(
                                new TurretNormalResetCommand(intakeSubsystem),
                                new IntakeSlidesInCommand(intakeSubsystem, transferSubsystem),
                                new WaitCommand(1000),
                                new ClawLooseCommand(intakeSubsystem)
                        )
                )
        );

        //Drop in human player zone
        operator.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new SequentialCommandGroup(
                        //Reset Transfer
                        new ParallelCommandGroup(
                                new IntakeSlidesOutCommand(intakeSubsystem),
                                new WaitCommand(500),
                                new IntakeClawOpenCommand(intakeSubsystem)
                        ),


                        //Reset Intake
                        new IntakeResetCommand(intakeSubsystem)
                )
        );

        //Pick Alliance Specific Sample after dropping specimen

        operator.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new TransferBackwardCommand(transferSubsystem),
                                new OpenGripplerCommand(transferSubsystem)
                        ),
                        //Reset Intake
                        new IntakeResetCommand(intakeSubsystem),
                        //Auto Aim one shot
                        new FireOneShotCommand(intakeSubsystem),
                        new WaitCommand(500),
                        new ClawCloseCommand(intakeSubsystem),
                        //Retract for Transfer
                        new IntakePivotUpCommand(intakeSubsystem, robotState),
                        new IntakeSlidesInCommand(intakeSubsystem, transferSubsystem),
                        new TurretSideTransferCommand(intakeSubsystem)
                )
        );

        //Drop in human player zone, reset Intake and Pick from Wall

        operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new IntakeClawOpenCommand(intakeSubsystem),
                                new TransferBackwardCommand(transferSubsystem)
                        ),
                        new WaitCommand(1000),
                        new IntakeResetCommand(intakeSubsystem)
                )

        ).whenReleased(
                new SequentialCommandGroup(
                        new WaitCommand(700),
                        new CloseGripplerCommand(transferSubsystem),
                        //Add Slides up
                        new SlidesHighChamberCommand(slidesSubsystem),
                        new TransferSpecimenDropCommand(transferSubsystem)
                )
        );

        //Drop Specimen, Intake and reset

        operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new OpenGripplerCommand(transferSubsystem),
                                new FireOneShotCommand(intakeSubsystem)
                        ),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new WaitCommand(500),
                                        new ClawCloseCommand(intakeSubsystem),
                                        //Retract for Transfer
                                        new IntakePivotUpCommand(intakeSubsystem, robotState),
                                        new IntakeSlidesInCommand(intakeSubsystem, transferSubsystem),
                                        new TurretSideTransferCommand(intakeSubsystem)
                                ),
                                new TransferBackwardCommand(transferSubsystem),
                                new SlidesStowCommand(slidesSubsystem)
                        )
                )

        );

        telemetry.addLine("A: extend/stow");
        telemetry.addLine("B: fire one-shot");
        telemetry.addLine("X: reset arm");
        telemetry.update();
    }
}
