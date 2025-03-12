package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotStateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlidesSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;

public class TeleOpIntakeCommand extends SequentialCommandGroup {

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final IntakeSubsystem intakeSubsystem;
    private final RobotStateSubsystem robotState;

    public TeleOpIntakeCommand(IntakeSubsystem Isubsystem, RobotStateSubsystem state) {
        intakeSubsystem = Isubsystem;
        robotState = state;

        addCommands(

                new IntakePivotUpCommand(intakeSubsystem, robotState),
                new IntakeSlidesOutCommand(intakeSubsystem),
                new WaitCommand(300),
                new PoopChuteOpenCommand(intakeSubsystem),
                new IntakePivotDownCommand(intakeSubsystem, robotState),
                new WaitCommand(500),
                new ColourAwareIntakeCommand(intakeSubsystem)

        );


        addRequirements(Isubsystem);
    }

    //DP: Notes
    //In a command group, we don't need to use the initalize function
    //we just add the commands in the constructor with the addCommands function
    //we also don't want to tell the scheduler that this command is finished.
    //instead, the sequential group command will use all of the added commands
    //to determine when it's finished.
    /*@Override
    public void initialize() {
        new SequentialCommandGroup(
           // new ParallelCommandGroup(
                new ColourAwareIntakeCommand(intakeSubsystem),
                new IntakeSlidesOutCommand(intakeSubsystem),
           // ),
                new WaitCommand(300),
            new IntakePivotDownCommand(intakeSubsystem)
        );
    }*/

    /*@Override
    public void execute(){
        intakeSubsystem.colourAwareIntake();
    }

    @Override
    public boolean isFinished() {
        return true;
    }*/
}
