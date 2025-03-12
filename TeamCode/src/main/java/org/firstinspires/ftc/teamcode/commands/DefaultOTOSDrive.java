package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OTOSDriveSubsystem;

import java.util.function.DoubleSupplier;

public class DefaultOTOSDrive extends CommandBase {

    private final OTOSDriveSubsystem m_drive;
    private final DoubleSupplier m_leftx;
    private final DoubleSupplier m_lefty;

    private final DoubleSupplier m_rightx;


    public DefaultOTOSDrive(OTOSDriveSubsystem subsystem, DoubleSupplier leftX, DoubleSupplier leftY, DoubleSupplier rightX) {
        m_drive = subsystem;
        m_leftx = leftX;
        m_lefty = leftY;
        m_rightx = rightX;
        addRequirements(m_drive);
    }

    @Override
    public void execute() {
        m_drive.drive(m_leftx.getAsDouble(), m_lefty.getAsDouble(), m_rightx.getAsDouble());
    }

}
