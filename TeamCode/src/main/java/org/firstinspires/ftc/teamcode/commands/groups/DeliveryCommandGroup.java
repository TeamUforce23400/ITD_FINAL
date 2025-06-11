package org.firstinspires.ftc.teamcode.commands.groups;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.SlidesHighBasketCommand;
import org.firstinspires.ftc.teamcode.commands.TransferFlipCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotStateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlidesSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;

public class DeliveryCommandGroup extends SequentialCommandGroup {

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final IntakeSubsystem intakeSubsystem;
    private final TransferSubsystem transferSubsystem;

    private final SlidesSubsystem slidesSubsystem;

    private final RobotStateSubsystem robotState;

    public DeliveryCommandGroup(IntakeSubsystem Isubsystem, TransferSubsystem Itransfer, SlidesSubsystem slides, RobotStateSubsystem state) {
        intakeSubsystem = Isubsystem;
        transferSubsystem = Itransfer;
        robotState = state;
        slidesSubsystem = slides;

        addCommands(
            new ParallelCommandGroup(
                    new TransferFlipCommand(transferSubsystem),
                    new SlidesHighBasketCommand(slidesSubsystem)
            )
        );


        addRequirements(Isubsystem);
        addRequirements(Itransfer);
    }
}