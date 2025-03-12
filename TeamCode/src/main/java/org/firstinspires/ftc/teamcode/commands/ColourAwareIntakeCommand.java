package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;

public class ColourAwareIntakeCommand extends CommandBase {

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final IntakeSubsystem intakeSubsystem;

    private ElapsedTime timer;

    private boolean pooping = false;


    public ColourAwareIntakeCommand(IntakeSubsystem subsystem) {
        intakeSubsystem = subsystem;
        timer = new ElapsedTime();
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute(){
        //start the colour aware intaking
        if(!pooping) {
            intakeSubsystem.poopChuteOpen();
            intakeSubsystem.colourAwareIntake();
        }
        if(intakeSubsystem.IsPooping()){
            if(intakeSubsystem.getCurrentIntakeColour() != IntakeSubsystem.SampleColour.NONE && intakeSubsystem.getDesiredIntakeColour() != intakeSubsystem.getCurrentIntakeColour()){
                //poop out

                pooping = true;
                intakeSubsystem.intakePivotUp();
                intakeSubsystem.poopChuteClose();


            }else{
                if(intakeSubsystem.getCurrentIntakeColour() == intakeSubsystem.getDesiredIntakeColour()){
                    intakeSubsystem.IntakeOff();
                    pooping = false;
                }
            }
            //delay
            if(pooping && timer.milliseconds() > 200) {
                intakeSubsystem.intakePivotDown();
                pooping = false;
                timer.reset();
            }
        }
    }

    @Override
    public boolean isFinished() {
        return intakeSubsystem.getCurrentIntakeColour() == intakeSubsystem.getDesiredIntakeColour();

    }
}
