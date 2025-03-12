package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class DesiredColourNeutralCommand extends CommandBase {

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final IntakeSubsystem intakeSubsystem;


    public DesiredColourNeutralCommand(IntakeSubsystem subsystem) {
        intakeSubsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        //turn outtake on
        intakeSubsystem.setDesiredColourNeutral();
    }

    @Override
    public boolean isFinished() {
        return intakeSubsystem.IsDesiredColourNeutralSet();
    }
}
