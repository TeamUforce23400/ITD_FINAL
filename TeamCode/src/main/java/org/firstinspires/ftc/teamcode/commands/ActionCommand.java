package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.Subsystem;
import java.util.Set;

public class ActionCommand implements Command {
    private final Runnable action;
    private final Set<Subsystem> requirements;
    private boolean finished = false;

    // Constructor
    public ActionCommand(Runnable action, Set<Subsystem> requirements) {
        this.action = action;
        this.requirements = requirements;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return requirements;
    }

    @Override
    public void initialize() {
        finished = false;
    }

    @Override
    public void execute() {
        action.run();
        finished = true;       // so isFinished() will return true next time
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
