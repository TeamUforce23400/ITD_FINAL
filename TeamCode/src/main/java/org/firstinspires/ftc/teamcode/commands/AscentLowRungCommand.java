package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.AscentSubsystem;

public class AscentLowRungCommand extends CommandBase {

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final AscentSubsystem ascentSubsystem;


    public AscentLowRungCommand(AscentSubsystem subsystem) {
        ascentSubsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        //Low rung ascent
        ascentSubsystem.ascentLowRung();
    }

    @Override
    public boolean isFinished() {
        return ascentSubsystem.IsAscentLowRung();
    }
}
