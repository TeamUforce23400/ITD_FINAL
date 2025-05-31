package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.CloseGripplerCommand;
import org.firstinspires.ftc.teamcode.commands.DefaultDrive;
import org.firstinspires.ftc.teamcode.commands.IntakePivotDownCommand;
import org.firstinspires.ftc.teamcode.commands.IntakePivotUpCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeSlidesInCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeSlidesOutCommand;
import org.firstinspires.ftc.teamcode.commands.OpenGripplerCommand;
import org.firstinspires.ftc.teamcode.commands.SlidesHighBasketCommand;
import org.firstinspires.ftc.teamcode.commands.SlidesStowCommand;
import org.firstinspires.ftc.teamcode.commands.TransferFlipCommand;
import org.firstinspires.ftc.teamcode.commands.TransferStowCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotStateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlidesSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;

@TeleOp(name = "Intake check")
public class Intakecheckclaw extends CommandOpMode {

    private DriveSubsystem      driveSubsystem;
    private DefaultDrive        defaultDriveCmd;
    private IntakeSubsystem     intakeSubsystem;
    private TransferSubsystem   transferSubsystem;
    private SlidesSubsystem     slidesSubsystem;
    private RobotStateSubsystem robotState;

    private GamepadEx driver;
    private GamepadEx operator;

    @Override
    public void initialize() {
        // 1) Gamepads
        driver   = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);


        intakeSubsystem   = new IntakeSubsystem(hardwareMap, telemetry);


        // 5) Button B: fire one-shot using last Limelight sample
        driver.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(intakeSubsystem::fireOneShot, intakeSubsystem));

        // 6) Button X: reset arm (raise pivots & resume sampling)
        driver.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new InstantCommand(intakeSubsystem::resetArm, intakeSubsystem));

        telemetry.addLine("A: extend/stow");
        telemetry.addLine("B: fire one-shot");
        telemetry.addLine("X: reset arm");
        telemetry.update();
    }
}
