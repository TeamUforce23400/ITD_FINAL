package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
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
@Disabled

@TeleOp(name = "Robot Inspection")

public class ExtendTeleOp extends CommandOpMode {

    private DriveSubsystem m_drive;
    private DefaultDrive m_driveCommand;
    private IntakeSubsystem intakeSubsystem;
    private TransferSubsystem transferSubsystem;
    private SlidesSubsystem slidesSubsystem;

    private RobotStateSubsystem robotState;
    private GamepadEx driver;
    private GamepadEx m_driveOperator;

    private double driveSpeed = 1;
    @Override
    public void initialize() {
        robotState = new RobotStateSubsystem();


        driver = new GamepadEx(gamepad1);
        m_driveOperator = new GamepadEx(gamepad2);

        intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetry);
        transferSubsystem = new TransferSubsystem(hardwareMap);
        slidesSubsystem = new SlidesSubsystem(hardwareMap, telemetry);

        telemetry.addData("BUTTON A TO EXTEND", "GO");
        telemetry.update();

        driver.getGamepadButton(GamepadKeys.Button.A).toggleWhenPressed(
            new SequentialCommandGroup(
                    new IntakeSlidesOutCommand(intakeSubsystem),
                    new WaitCommand(300),
                    new IntakePivotDownCommand(intakeSubsystem, robotState),
                    new SlidesHighBasketCommand(slidesSubsystem),
                    new TransferFlipCommand(transferSubsystem),
                    new CloseGripplerCommand(transferSubsystem)
            ),
            new SequentialCommandGroup(
                    new OpenGripplerCommand(transferSubsystem),
                    new IntakePivotUpCommand(intakeSubsystem, robotState),
                    new IntakeSlidesInCommand(intakeSubsystem, transferSubsystem),
                    new TransferStowCommand(transferSubsystem),
                    new SlidesStowCommand(slidesSubsystem)
            )
        );





    }
}


