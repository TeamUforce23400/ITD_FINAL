package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.AscentSubsystem;

public class AscentOpenHooksCommand extends CommandBase {

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final AscentSubsystem ascentSubsystem;


    public AscentOpenHooksCommand(AscentSubsystem subsystem) {
        ascentSubsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        //High rung ascent
        ascentSubsystem.ascentOpenHooks();
    }

    @Override
    public boolean isFinished() {
        return ascentSubsystem.AreAscentHooksOpen();
    }
}
