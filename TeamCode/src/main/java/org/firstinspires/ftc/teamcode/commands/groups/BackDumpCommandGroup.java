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
import org.firstinspires.ftc.teamcode.commands.OpenGripplerCommand;
import org.firstinspires.ftc.teamcode.commands.PoopChuteCloseCommand;
import org.firstinspires.ftc.teamcode.commands.SlidesDumpCommand;
import org.firstinspires.ftc.teamcode.commands.SlidesHalfDumpCommand;
import org.firstinspires.ftc.teamcode.commands.SlidesStowCommand;
import org.firstinspires.ftc.teamcode.commands.SlowIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.TransferBackwardCommand;
import org.firstinspires.ftc.teamcode.commands.TransferStowCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotStateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlidesSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;

public class BackDumpCommandGroup extends SequentialCommandGroup {

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final SlidesSubsystem slidesSubsystem;
    private final TransferSubsystem transferSubsystem;

    private final RobotStateSubsystem robotState;

    public BackDumpCommandGroup(SlidesSubsystem Islides, TransferSubsystem Itransfer, RobotStateSubsystem state) {
        slidesSubsystem = Islides;
        transferSubsystem = Itransfer;
        robotState = state;

        addCommands(
                new CloseGripplerCommand(transferSubsystem),
                new SlidesDumpCommand(slidesSubsystem),
                new TransferBackwardCommand(transferSubsystem),
                new WaitCommand(200),
                new SlidesHalfDumpCommand(slidesSubsystem),
                new WaitCommand(200),
                new OpenGripplerCommand(transferSubsystem),
                new WaitCommand(200),
                new ParallelCommandGroup(
                    new SequentialCommandGroup(
                        new CloseGripplerCommand(transferSubsystem),
                        new WaitCommand(300)
                            ),
                    new SlidesDumpCommand(slidesSubsystem)
                ),
                new TransferStowCommand(transferSubsystem),
                new WaitCommand(200),
                new SlidesStowCommand(slidesSubsystem)
        );


        addRequirements(Islides);
        addRequirements(Itransfer);
    }
}