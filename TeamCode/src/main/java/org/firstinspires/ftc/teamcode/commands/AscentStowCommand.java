package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.AscentSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class AscentStowCommand extends CommandBase {

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final AscentSubsystem ascentSubsystem;


    public AscentStowCommand(AscentSubsystem subsystem) {
        ascentSubsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        //Stow ascent
        ascentSubsystem.ascentStow();
    }

    @Override
    public boolean isFinished() {
        return ascentSubsystem.IsAscentStowed();
    }
}
