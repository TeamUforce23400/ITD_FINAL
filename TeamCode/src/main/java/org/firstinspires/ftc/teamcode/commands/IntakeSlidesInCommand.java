package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;

public class IntakeSlidesInCommand extends CommandBase {

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final IntakeSubsystem intakeSubsystem;

    private final TransferSubsystem transferSubsystem;


    public IntakeSlidesInCommand(IntakeSubsystem subsystem, TransferSubsystem transfer) {
        intakeSubsystem = subsystem;
        transferSubsystem = transfer;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        //turn outtake on
        intakeSubsystem.intakeSlidesIn();
    }

    @Override
    public boolean isFinished() {
        return transferSubsystem.IsTransferClosed();
    }
}
