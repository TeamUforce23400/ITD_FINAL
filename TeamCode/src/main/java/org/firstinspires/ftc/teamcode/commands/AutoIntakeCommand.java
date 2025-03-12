package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlidesSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;

public class AutoIntakeCommand extends SequentialCommandGroup {

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final IntakeSubsystem intakeSubsystem;
    private final TransferSubsystem transferSubsystem;
    private final SlidesSubsystem slidesSubsystem;

    public AutoIntakeCommand(IntakeSubsystem Isubsystem, TransferSubsystem Tsubsystem, SlidesSubsystem Ssubsystem) {
        intakeSubsystem = Isubsystem;
        transferSubsystem = Tsubsystem;
        slidesSubsystem = Ssubsystem;


        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(Isubsystem);
        addRequirements(Tsubsystem);
        addRequirements(Ssubsystem);
    }

    @Override
    public void initialize() {
        new ParallelCommandGroup(
            new ColourAwareIntakeCommand(intakeSubsystem),
            new IntakeSlidesOutCommand(intakeSubsystem)
        );
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
