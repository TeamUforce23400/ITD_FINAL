package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class AutoOnlyIntakeCommand extends CommandBase {

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final IntakeSubsystem intakeSubsystem;

    private ElapsedTime timer;

    private boolean pooping = false;


    public AutoOnlyIntakeCommand(IntakeSubsystem subsystem) {
        intakeSubsystem = subsystem;
        timer = new ElapsedTime();
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {

    }


    @Override
    public boolean isFinished() {
        return intakeSubsystem.getCurrentIntakeColour() == intakeSubsystem.getDesiredIntakeColour();

    }
}
