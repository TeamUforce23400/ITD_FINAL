package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@Config
public class SlidesSubsystem extends SubsystemBase {

    //Define motors and servos
    private DcMotor verticalSlideMotor1;
    private DcMotor verticalSlideMotor2;

    // Define variables
    private int stowedSlidesPosition = 0;
    private int backwardsTransferPosition = 0;
    private int lowChamberPosition = 150;
    private int highChamberPosition = 500;
    private int lowBasketPosition = 800;
    private int highBasketPosition = 2150;

    private int dumpPosition = 800;

    private int dumpHalfPos = 400;

    private int deliverHighChamberPosition = 0;

    //IMPORTANT: this value gives the motor some breathing room on the retraction
    //we can cook motors if this value is too LOW
    //prob only want to make this number larger than 50
    private int STOWED_SLIDE_DIFFERENCE = 50;



    public double target;

    private Telemetry telemetry;

    public static double kP = 0.01;
    public static double kI = 0.0;
    public static double kD = 0.0002;
    public static double kF = 0.00018;

    private static final PIDFController slidePIDF = new PIDFController(kP, kI, kD, kF);




    public SlidesSubsystem(final HardwareMap hMap,  Telemetry telemetry) {
        verticalSlideMotor1 = hMap.get(DcMotor.class, "cr");
        verticalSlideMotor2 = hMap.get(DcMotor.class, "cl");
        verticalSlideMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        verticalSlideMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Target Position", target);
        telemetry.addData("Current Position Right", verticalSlideMotor1.getCurrentPosition());
        telemetry.addData("Current Position Left", verticalSlideMotor2.getCurrentPosition());
        telemetry.update();


        resetVerticalSlides();

        this.telemetry = telemetry;
    }


    public void resetVerticalSlides(){
        verticalSlideMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalSlideMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void JoySlideUp(){
        verticalSlideMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        verticalSlideMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        verticalSlideMotor1.setPower(1.0);
        verticalSlideMotor2.setPower(1.0);
    }

    public void JoySlideDown(){
        verticalSlideMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        verticalSlideMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        verticalSlideMotor1.setPower(-1.0);
        verticalSlideMotor2.setPower(-1.0);
    }


    public void stowSlides() {
        //verticalSlideMotor.setTargetPosition(stowedSlidesPosition);
        //verticalSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //verticalSlideMotor.setPower(1);
        setSlideTarget(stowedSlidesPosition);
    }

    public void NoPowerSlides(){
        verticalSlideMotor1.setPower(0);
        verticalSlideMotor2.setPower(0);
    }

    public double getCurrentSlidePos(){
        return verticalSlideMotor1.getCurrentPosition();

    }

    public boolean AreSlidesStowed() {
        return verticalSlideMotor1.getCurrentPosition() < (stowedSlidesPosition + STOWED_SLIDE_DIFFERENCE);
    }

    public void highChamberDeliver(){
//        verticalSlideMotor1.setTargetPosition(deliverHighChamberPosition);
//        verticalSlideMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        verticalSlideMotor1.setPower(1);
//
//        verticalSlideMotor2.setTargetPosition(deliverHighChamberPosition);
//        verticalSlideMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        verticalSlideMotor2.setPower(1);

        setSlideTarget(deliverHighChamberPosition);
    }

    public boolean IsAtHighChamberDeliver(){
        return verticalSlideMotor1.getCurrentPosition() < (deliverHighChamberPosition +50);
    }

    public void backwardsTransfer() {
        verticalSlideMotor1.setTargetPosition(backwardsTransferPosition);
        verticalSlideMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalSlideMotor1.setPower(1);

        verticalSlideMotor2.setTargetPosition(backwardsTransferPosition);
        verticalSlideMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalSlideMotor2.setPower(1);
    }

    public boolean AreSlidesAllowingBackwardsTransfer() {
        return verticalSlideMotor1.getCurrentPosition() > (backwardsTransferPosition - 50);
    }

    public void lowChamber() {
        verticalSlideMotor1.setTargetPosition(lowChamberPosition);
        verticalSlideMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalSlideMotor1.setPower(1);

        verticalSlideMotor2.setTargetPosition(lowChamberPosition);
        verticalSlideMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalSlideMotor2.setPower(1);
    }

    public boolean IsAtLowChamber() {
        return verticalSlideMotor1.getCurrentPosition() > (lowChamberPosition - 50);
    }
    public void highChamber() {
//        verticalSlideMotor1.setTargetPosition(highChamberPosition);
//        verticalSlideMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        verticalSlideMotor1.setPower(1);
//
//        verticalSlideMotor2.setTargetPosition(highChamberPosition);
//        verticalSlideMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        verticalSlideMotor2.setPower(1);

        setSlideTarget(highChamberPosition);
    }

    public boolean IsAtHighChamber() {
        return verticalSlideMotor1.getCurrentPosition() > (highChamberPosition - 50);
    }

    public void lowBasket() {
        /*verticalSlideMotor.setTargetPosition(lowBasketPosition);
        verticalSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalSlideMotor.setPower(1);*/
        setSlideTarget(lowBasketPosition);
    }

    public void dumpPosition(){
        setSlideTarget(dumpPosition);
    }

    public boolean IsAtDumpPosition(){
        return verticalSlideMotor1.getCurrentPosition() > (dumpPosition - 50);
    }

    public void halfDump(){
        setSlideTarget(dumpHalfPos);
    }

    public boolean IsAtHalfDumpPosition(){
        return verticalSlideMotor1.getCurrentPosition() > (dumpHalfPos - 50);
    }

    public boolean IsAtLowBasket() {
        return verticalSlideMotor1.getCurrentPosition() > (lowBasketPosition - 50);
    }
    public void highBasket() {
        /*
        verticalSlideMotor.setTargetPosition(highBasketPosition);
        verticalSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalSlideMotor.setPower(1);
        */
         setSlideTarget(highBasketPosition);
    }

    public boolean IsAtHighBasket() {
        return verticalSlideMotor1.getCurrentPosition() > (highBasketPosition - 50);
    }

    public void setSlideTarget(double target) {
        this.target = Math.max(Math.min(target, highBasketPosition), 0);
        slidePIDF.setSetPoint(target);
    }

    public void autoUpdateSlides() {
        slidePIDF.setPIDF(kP, kI, kD, kF);

        double avgPosition = (verticalSlideMotor1.getCurrentPosition() + verticalSlideMotor2.getCurrentPosition()) / 2.0;
        double power = slidePIDF.calculate(avgPosition, target);

        telemetry.addData("Target Position", target);
        telemetry.addData("Current Position", verticalSlideMotor1.getCurrentPosition());
        telemetry.addData("Motor Power", power);
        telemetry.update();

        verticalSlideMotor1.setPower(power);
        verticalSlideMotor2.setPower(power);

    }

    @Override
    public void periodic() {
        autoUpdateSlides();
    }
}