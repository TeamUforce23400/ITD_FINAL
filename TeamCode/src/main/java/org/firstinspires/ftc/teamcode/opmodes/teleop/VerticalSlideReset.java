package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.CloseGripplerCommand;
import org.firstinspires.ftc.teamcode.commands.DefaultOTOSDrive;
import org.firstinspires.ftc.teamcode.commands.OpenGripplerCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OTOSDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlidesSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;
@Disabled
@TeleOp(name = "Vertical Slide Reset")
public class VerticalSlideReset extends CommandOpMode {


    private SlidesSubsystem slidesSubsystem;

    private GamepadEx m_driverOp;
    @Override
    public void initialize() {

        slidesSubsystem = new SlidesSubsystem(hardwareMap, telemetry);
        m_driverOp = new GamepadEx(gamepad1);

        slidesSubsystem.resetVerticalSlides();

        telemetry.addData("SLIDES", "RESET COMPLETE");
        telemetry.update();

    }
}


