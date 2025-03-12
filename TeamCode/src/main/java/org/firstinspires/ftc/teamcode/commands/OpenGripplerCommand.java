package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;

public class OpenGripplerCommand extends CommandBase {

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final TransferSubsystem transferSubsystem;


    public OpenGripplerCommand(TransferSubsystem subsystem) {
        transferSubsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        transferSubsystem.openGrippler();
    }

    @Override
    public boolean isFinished() {
        return transferSubsystem.IsGripplerOpen();
    }
}
