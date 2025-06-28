package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.Subsystem;
import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.PathChain;
import java.util.Collections;
import java.util.Set;

public class FollowPathCommand implements Command {
    private final Follower follower;
    private final PathChain path;
    private final boolean async;

    public FollowPathCommand(Follower follower, PathChain path, boolean async) {
        this.follower = follower;
        this.path     = path;
        this.async    = async;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Collections.emptySet();
    }

    @Override
    public void initialize() {
        follower.followPath(path, async);
    }

    @Override
    public void execute() {
        // nothing needed here; follower.update() is called in your loop()
    }

    @Override
    public boolean isFinished() {
        return !follower.isBusy();
    }
}
