package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.AscentSubsystem;

public class AscentFinishLevel3Command extends CommandBase {

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final AscentSubsystem ascentSubsystem;


    public AscentFinishLevel3Command(AscentSubsystem subsystem) {
        ascentSubsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        //High rung ascent
        ascentSubsystem.ascentFinishLevel3();
    }

    @Override
    public boolean isFinished() {
        return ascentSubsystem.IsAscentFinishedLevel3();
    }
}
