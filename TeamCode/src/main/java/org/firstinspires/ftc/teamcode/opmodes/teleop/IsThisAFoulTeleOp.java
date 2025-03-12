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
import org.firstinspires.ftc.teamcode.commands.IntakePoopChuteCloseCommand;
import org.firstinspires.ftc.teamcode.commands.IntakePoopChuteOpenCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeSlidesInCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeSlidesOutCommand;
import org.firstinspires.ftc.teamcode.commands.OpenGripplerCommand;
import org.firstinspires.ftc.teamcode.commands.OuttakeOnCommand;
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

import java.util.function.BooleanSupplier;

@TeleOp(name = "Is this a foul?")
@Disabled
public class IsThisAFoulTeleOp extends CommandOpMode {
// rightn trigger: basket
    // left trigger slides out and intake
    // right bumper speciman chamber
    private DriveSubsystem m_drive;
    private DefaultDrive m_driveCommand;
    private IntakeSubsystem intakeSubsystem;
    private RobotStateSubsystem robotState;
    private GamepadEx m_driveDriver;
    private GamepadEx m_driveOperator;

    private double driveSpeed = 0.4;


    @Override
    public void initialize() {
        m_drive = new DriveSubsystem(hardwareMap, telemetry);

        m_driveDriver = new GamepadEx(gamepad1);
        m_driveOperator = new GamepadEx(gamepad2);

        intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetry);
        //SETUP the starting COLOUR:
        intakeSubsystem.setDesiredColour(IntakeSubsystem.SampleColour.BLUE_OR_NEUTRAL);

        m_driveCommand = new DefaultDrive(m_drive, () -> m_driveDriver.getLeftX(),  () -> m_driveDriver.getLeftY(), () -> m_driveDriver.getRightX() * 0.5 , ()-> driveSpeed);

        robotState = new RobotStateSubsystem();

        register(m_drive);
        m_drive.setDefaultCommand(m_driveCommand);

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
        m_driveDriver.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new SequentialCommandGroup(
                        new IntakePivotDownCommand(intakeSubsystem, robotState),
                        new IntakePoopChuteCloseCommand(intakeSubsystem),
                        new IntakeSlidesOutCommand(intakeSubsystem),
                        new IntakeOnCommand(intakeSubsystem)
                )

        );
    }
}


