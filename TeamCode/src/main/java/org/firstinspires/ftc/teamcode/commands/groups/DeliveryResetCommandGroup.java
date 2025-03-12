package org.firstinspires.ftc.teamcode.commands.groups;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.OpenGripplerCommand;
import org.firstinspires.ftc.teamcode.commands.SlidesHighBasketCommand;
import org.firstinspires.ftc.teamcode.commands.SlidesStowCommand;
import org.firstinspires.ftc.teamcode.commands.TransferFlipCommand;
import org.firstinspires.ftc.teamcode.commands.TransferStowCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotStateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlidesSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;

public class DeliveryResetCommandGroup extends SequentialCommandGroup {

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final IntakeSubsystem intakeSubsystem;
    private final TransferSubsystem transferSubsystem;

    private final SlidesSubsystem slidesSubsystem;

    private final RobotStateSubsystem robotState;

    public DeliveryResetCommandGroup(IntakeSubsystem Isubsystem, TransferSubsystem Itransfer, SlidesSubsystem slides, RobotStateSubsystem state) {
        intakeSubsystem = Isubsystem;
        transferSubsystem = Itransfer;
        robotState = state;
        slidesSubsystem = slides;

        addCommands(
            new ParallelCommandGroup(
                    new TransferStowCommand(transferSubsystem),
                    new SlidesStowCommand(slidesSubsystem)
            ),
                new OpenGripplerCommand(transferSubsystem)
        );


        addRequirements(Isubsystem);
        addRequirements(Itransfer);
    }
}