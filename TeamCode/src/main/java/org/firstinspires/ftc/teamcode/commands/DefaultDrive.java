package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;


import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class DefaultDrive  extends CommandBase {

    private final DriveSubsystem m_drive;
    private final DoubleSupplier m_leftx;
    private final DoubleSupplier m_lefty;

    private final DoubleSupplier m_rightx;

    private final DoubleSupplier m_scale;

    public DefaultDrive(DriveSubsystem subsystem, DoubleSupplier leftX, DoubleSupplier leftY, DoubleSupplier rightX, DoubleSupplier scale) {
        m_drive = subsystem;
        m_leftx = leftX;
        m_lefty = leftY;
        m_rightx = rightX;
        m_scale = scale;
        addRequirements(m_drive);
    }

    @Override
    public void execute() {
        m_drive.drive(m_leftx.getAsDouble(), m_lefty.getAsDouble(), m_rightx.getAsDouble(), m_scale.getAsDouble());
    }

}
