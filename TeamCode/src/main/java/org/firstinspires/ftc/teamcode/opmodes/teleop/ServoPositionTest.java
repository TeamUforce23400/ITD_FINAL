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
import org.firstinspires.ftc.teamcode.commands.DefaultDrive;
import org.firstinspires.ftc.teamcode.commands.FireOneShotCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeClawOpenCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeClawYawBaseCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeClawYawSecondCommand;
import org.firstinspires.ftc.teamcode.commands.IntakePivotDownCommand;
import org.firstinspires.ftc.teamcode.commands.IntakePivotUpCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeSlidesInCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeSlidesOutCommand;
import org.firstinspires.ftc.teamcode.commands.ResetArmForIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.TurretResetTransferCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotStateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;

@TeleOp(name = "Servo Position Test")
public class ServoPositionTest extends CommandOpMode {

//    private DriveSubsystem m_drive;
//    private DefaultDrive m_driveCommand;
    private IntakeSubsystem     intakeSubsystem;
    private TransferSubsystem   transferSubsystem;
//    private SlidesSubsystem     slidesSubsystem;
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
//
//        slidesSubsystem = new SlidesSubsystem(hardwareMap, telemetry);
        robotState = new RobotStateSubsystem();
//        m_drive = new DriveSubsystem(hardwareMap, telemetry);

//        m_driveCommand = new DefaultDrive(m_drive, () -> driver.getLeftX(),  () -> driver.getLeftY(), () -> driver.getRightX() * 0.5 , ()-> driveSpeed);
//
//
//        register(m_drive);
//        m_drive.setDefaultCommand(m_driveCommand);

        //Sample Intake

        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new FireOneShotCommand(intakeSubsystem)
        );

        driver.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new SequentialCommandGroup(
                        new ClawCloseCommand(intakeSubsystem),
                        //Retract for Transfer
                        new IntakePivotUpCommand(intakeSubsystem, robotState),

                                new TurretResetTransferCommand(intakeSubsystem),
                                new IntakeSlidesInCommand(intakeSubsystem, transferSubsystem),
                                new WaitCommand(1000),
                                new ClawLooseCommand(intakeSubsystem)

                )
        );



        driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new ResetArmForIntakeCommand(intakeSubsystem)
        );

        driver.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new TurretResetTransferCommand(intakeSubsystem)
        );

        driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                new IntakePivotUpCommand(intakeSubsystem, robotState)
        );

        driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                new IntakePivotUpCommand(intakeSubsystem, robotState)
        );

        driver.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new IntakePivotDownCommand(intakeSubsystem, robotState)
        );



        telemetry.addLine("A: extend/stow");
        telemetry.addLine("B: fire one-shot");
        telemetry.addLine("X: reset arm");
        telemetry.update();
    }
}
