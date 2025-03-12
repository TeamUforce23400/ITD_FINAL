package org.firstinspires.ftc.teamcode.commands.groups;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.CloseGripplerCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeOffCommand;
import org.firstinspires.ftc.teamcode.commands.IntakePivotDownCommand;
import org.firstinspires.ftc.teamcode.commands.IntakePivotUpCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeSlidesInCommand;
import org.firstinspires.ftc.teamcode.commands.MiddleGripplerRotationCommand;
import org.firstinspires.ftc.teamcode.commands.PoopChuteCloseCommand;
import org.firstinspires.ftc.teamcode.commands.SlowIntakeCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotStateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;

public class AutoSpecimenCommandGroup extends SequentialCommandGroup {

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final IntakeSubsystem intakeSubsystem;
    private final TransferSubsystem transferSubsystem;

    private final RobotStateSubsystem robotState;

    public AutoSpecimenCommandGroup(IntakeSubsystem Isubsystem, TransferSubsystem Itransfer, RobotStateSubsystem state) {
        intakeSubsystem = Isubsystem;
        transferSubsystem = Itransfer;
        robotState = state;

        addCommands(

                new IntakeOffCommand(intakeSubsystem),

                new ParallelCommandGroup(
                        new IntakePivotUpCommand(intakeSubsystem, robotState),
                        new MiddleGripplerRotationCommand(transferSubsystem)
                ),

                new IntakeSlidesInCommand(intakeSubsystem, transferSubsystem).withTimeout(500),
                new WaitCommand(100), //wait for the transfer to stablize
//                new SlowIntakeCommand(intakeSubsystem),
                new IntakeOffCommand(intakeSubsystem),
                new ConditionalCommand(
                        new WaitCommand(1), //do nothing, we have no sample
                        new SequentialCommandGroup(
                             //   new CloseGripplerCommand(transferSubsystem),
                                new WaitCommand(180),
                                new IntakePivotDownCommand(intakeSubsystem, robotState)),
                        () -> {return intakeSubsystem.getCurrentIntakeColour() == IntakeSubsystem.SampleColour.NONE;}
                )

        );


        addRequirements(Isubsystem);
        addRequirements(Itransfer);
    }
}