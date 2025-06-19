package org.firstinspires.ftc.teamcode.commands.groups;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.ClawCloseCommand;
import org.firstinspires.ftc.teamcode.commands.ClawLooseCommand;
import org.firstinspires.ftc.teamcode.commands.CloseGripplerCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeClawOpenCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeClawYawBaseCommand;
import org.firstinspires.ftc.teamcode.commands.IntakePivotDownCommand;
import org.firstinspires.ftc.teamcode.commands.IntakePivotIntakePosCommand;
import org.firstinspires.ftc.teamcode.commands.IntakePivotUpCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeSlidesInTransferCommand;
import org.firstinspires.ftc.teamcode.commands.OpenGripplerCommand;
import org.firstinspires.ftc.teamcode.commands.SlidesDumpCommand;
import org.firstinspires.ftc.teamcode.commands.SlidesHalfDumpCommand;
import org.firstinspires.ftc.teamcode.commands.SlidesStowCommand;
import org.firstinspires.ftc.teamcode.commands.TransferBackwardCommand;
import org.firstinspires.ftc.teamcode.commands.TransferSpecPreDrop;
import org.firstinspires.ftc.teamcode.commands.TransferStowCommand;
import org.firstinspires.ftc.teamcode.commands.TurretNormalResetCommand;
import org.firstinspires.ftc.teamcode.commands.TurretResetTransferCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotStateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlidesSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;

public class RetractCommandGroup extends SequentialCommandGroup {

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final SlidesSubsystem slidesSubsystem;
    private final TransferSubsystem transferSubsystem;

    private final RobotStateSubsystem robotState;
    private final IntakeSubsystem intakeSubsystem;

    public RetractCommandGroup(SlidesSubsystem Islides, TransferSubsystem Itransfer, RobotStateSubsystem state, IntakeSubsystem intake ) {
        slidesSubsystem = Islides;
        transferSubsystem = Itransfer;
        robotState = state;
        intakeSubsystem = intake;

        addCommands(
                new TransferStowCommand(transferSubsystem),
                new IntakePivotDownCommand(intakeSubsystem, robotState),
                new WaitCommand(300),
                new ClawCloseCommand(intakeSubsystem),

                new IntakePivotIntakePosCommand(intakeSubsystem, robotState),
                new IntakePivotUpCommand(intakeSubsystem, robotState),
                new WaitCommand(200),
                new TurretNormalResetCommand(intakeSubsystem),
                new IntakeClawYawBaseCommand(intakeSubsystem),

                new WaitCommand(700),
                new TurretResetTransferCommand(intakeSubsystem),
                new WaitCommand(400),
                new IntakeSlidesInTransferCommand(intakeSubsystem, transferSubsystem),
//                new ClawLooseCommand(intakeSubsystem),
                new WaitCommand(200)

//                new InstantCommand(()->intakeSubsystem.intakeSlidesFrontTransfer()),
//                new TurretNormalResetCommand(intakeSubsystem),
//                new IntakePivotIntakePosCommand(intakeSubsystem, robotState)

        );


        addRequirements(Islides);
        addRequirements(Itransfer);
        addRequirements(intake);

    }
}