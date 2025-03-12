package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.SlidesSubsystem;

public class SlidesHalfDumpCommand extends CommandBase {

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final SlidesSubsystem slidesSubsystem;


    public SlidesHalfDumpCommand(SlidesSubsystem subsystem) {
        slidesSubsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        slidesSubsystem.halfDump();
    }

    @Override
    public boolean isFinished() {
        return slidesSubsystem.IsAtHalfDumpPosition();
    }
}
