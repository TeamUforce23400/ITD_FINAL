package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.SlidesSubsystem;

@Config
@TeleOp(name = "Slides PID Dashboard Tuner")
public class SlidesDashboardTunerOpMode extends OpMode {
    private SlidesSubsystem slides;
    private MultipleTelemetry dashboardTelemetry;

    @Override
    public void init() {
        slides = new SlidesSubsystem(hardwareMap, telemetry);
        // Initialize dashboard telemetry
        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        dashboardTelemetry.addLine("Slides Dashboard Tuner Initialized");
        dashboardTelemetry.update();
    }

    @Override
    public void loop() {
        // Use the targetPosition configured in SlidesSubsystem via Dashboard
        slides.setSlideTarget(SlidesSubsystem.targetPosition);
        slides.autoUpdateSlides();

        // Stream values to dashboard
        dashboardTelemetry.addData("Target Position", SlidesSubsystem.targetPosition);
        dashboardTelemetry.addData("Current Position", slides.getCurrentSlidePos());
        dashboardTelemetry.addData("kP", SlidesSubsystem.kP);
        dashboardTelemetry.addData("kI", SlidesSubsystem.kI);
        dashboardTelemetry.addData("kD", SlidesSubsystem.kD);
        dashboardTelemetry.addData("kF", SlidesSubsystem.kF);
        dashboardTelemetry.update();
    }
}

//@Override
//public void loop() {
//    // Apply the dashboard-configured target position
//    slides.setSlideTarget(targetPosition);
//    // Run PIDF update on slides
//    slides.autoUpdateSlides();
//
//    // Stream values to dashboard
//    dashboardTelemetry.addData("Target Position", targetPosition);
//    dashboardTelemetry.addData("Current Position", slides.getCurrentSlidePos());
//    dashboardTelemetry.addData("kP", SlidesSubsystem.kP);
//    dashboardTelemetry.addData("kI", SlidesSubsystem.kI);
//    dashboardTelemetry.addData("kD", SlidesSubsystem.kD);
//    dashboardTelemetry.addData("kF", SlidesSubsystem.kF);
//    dashboardTelemetry.update();
//}
