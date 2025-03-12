package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotStateSubsystem;

public class IntakePivotUpCommand extends CommandBase {

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final IntakeSubsystem intakeSubsystem;
    private final RobotStateSubsystem robotState;


    public IntakePivotUpCommand(IntakeSubsystem subsystem, RobotStateSubsystem state) {
        intakeSubsystem = subsystem;
        robotState = state;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        //turn outtake on
        intakeSubsystem.intakePivotUp();
        robotState.pivotPosition = RobotStateSubsystem.PivotState.HIGH;
    }

    @Override
    public boolean isFinished() {
        return intakeSubsystem.IsIntakePivotedUp();
    }
}
