package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotStateSubsystem;

public class IntakePivotDownCommand extends CommandBase {

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final IntakeSubsystem intakeSubsystem;
    private final RobotStateSubsystem robotState;


    public IntakePivotDownCommand(IntakeSubsystem subsystem, RobotStateSubsystem state) {
        intakeSubsystem = subsystem;
        robotState = state;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        //turn outtake on
        intakeSubsystem.intakePivotDown();
        robotState.pivotPosition = RobotStateSubsystem.PivotState.LOW;
    }

    @Override
    public boolean isFinished() {
        return intakeSubsystem.IsIntakePivotedDown();
    }
}
