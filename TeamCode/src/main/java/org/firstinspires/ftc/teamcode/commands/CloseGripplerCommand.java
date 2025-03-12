package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;

public class CloseGripplerCommand extends CommandBase {

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final TransferSubsystem transferSubsystem;


    public CloseGripplerCommand(TransferSubsystem subsystem) {
        transferSubsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        transferSubsystem.closeGrippler();
    }

    @Override
    public boolean isFinished() {
        return transferSubsystem.IsGripplerClosed();
    }
}
