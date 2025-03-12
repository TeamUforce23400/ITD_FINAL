package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.SlidesSubsystem;

public class SlidesHighChamberCommand extends CommandBase {

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final SlidesSubsystem slidesSubsystem;


    public SlidesHighChamberCommand(SlidesSubsystem subsystem) {
        slidesSubsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        slidesSubsystem.highChamber();
    }

    @Override
    public boolean isFinished() {
        return slidesSubsystem.IsAtHighChamber();
    }
}
