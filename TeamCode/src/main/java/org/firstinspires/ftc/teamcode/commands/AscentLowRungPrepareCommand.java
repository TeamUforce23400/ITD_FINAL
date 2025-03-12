package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.AscentSubsystem;

public class AscentLowRungPrepareCommand extends CommandBase {

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final AscentSubsystem ascentSubsystem;


    public AscentLowRungPrepareCommand(AscentSubsystem subsystem) {
        ascentSubsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        //Prepare low rung ascent
        ascentSubsystem.ascentLowRungPrepare();
    }

    @Override
    public boolean isFinished() {
        return ascentSubsystem.IsAscentPrepared();
    }
}
