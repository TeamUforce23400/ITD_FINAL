package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.SlidesSubsystem;

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


