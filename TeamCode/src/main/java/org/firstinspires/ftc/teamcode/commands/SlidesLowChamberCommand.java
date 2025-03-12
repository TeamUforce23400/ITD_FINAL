package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.SlidesSubsystem;

public class SlidesLowChamberCommand extends CommandBase {

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final SlidesSubsystem slidesSubsystem;


    public SlidesLowChamberCommand(SlidesSubsystem subsystem) {
        slidesSubsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        slidesSubsystem.lowChamber();
    }

    @Override
    public boolean isFinished() {
        return slidesSubsystem.IsAtLowChamber();
    }
}
