package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;

public class MiddleGripplerRotationCommand extends CommandBase {

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final TransferSubsystem transferSubsystem;


    public MiddleGripplerRotationCommand(TransferSubsystem subsystem) {
        transferSubsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        transferSubsystem.middleGripplerRotation();
    }

    @Override
    public boolean isFinished() {
        return transferSubsystem.IsGripplerAtMiddleRotation();
    }
}
