package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.opencv.core.Point3;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeSubsystem extends SubsystemBase {

    //Define motors and servos
    private DcMotor intakeMotor;

    private Servo intakePivot;
//    private Servo intakeRightPivot;
    private Servo intakeClaw;
    public static Servo intakeLeftSlide;
    public static Servo intakeRightSlide;

    private Servo poopChute;

    private DigitalChannel colorPin0;

    private DigitalChannel colorPin1;


    // Define the colour sensor
    private RevColorSensorV3 colourSensor;

    // Define variables
    private double intakeSlidesInPosition = 0.65;

    private double intakeSlidesOutPosition = 0.06;

    private double intakePivotUpPosition = 0.2;
    private double intakeIntakePos = 0.45;
    private double intakePivotDownPosition = 1.0;

    private double intakePoopOpen = 0.85;

    private double intakePoopClose = 0.3;

    private double turretRest = 0.59;
    //0.59
    private double turretSide = 0.28;

    private double turretTransfer = 0.017;
//    private final float[] hsvValues = new float[3];
//
//    private SampleColour desiredColour = SampleColour.NEUTRAL;

//    public enum SampleColour
//    {
//        NONE,
//        RED,
//        BLUE,
//        NEUTRAL,
//
//        RED_OR_NEUTRAL,
//
//        BLUE_OR_NEUTRAL,
//
//        AUTO_ANY
//    }

    private boolean scanningEnabled = true;      // only sample when true
    private Point3 lastTarget;
    private double lastYaw;

    private Telemetry telemetry;

    private boolean isPooping = true;

    public static Servo turretServo;
    public static Servo clawYawServo;

    private final LimelightDetection limelight;


    public IntakeSubsystem(final HardwareMap hMap, Telemetry telemetry){

        this.telemetry = telemetry;

//        intakeMotor = hMap.get(DcMotor.class, "intake");

        intakePivot = hMap.get(Servo.class, "pl");
//        intakeRightPivot = hMap.get(Servo.class, "pr");
        intakeLeftSlide = hMap.get(Servo.class, "ll");
        intakeRightSlide = hMap.get(Servo.class, "rl");
//        poopChute = hMap.get(Servo.class, "pc");

        turretServo = hMap.get(Servo.class, "turret");
        clawYawServo = hMap.get(Servo.class, "wrist");
        intakeClaw = hMap.get(Servo.class, "intakeclaw");

        limelight = new LimelightDetection(hMap, telemetry);

//        intakeMotor.setDirection(DcMotor.Direction.REVERSE);

//        colourSensor = hMap.get(RevColorSensorV3.class, "cs");

        //TODO: Test fast mode
//        ((LynxI2cDeviceSynch) colourSensor.getDeviceClient()).setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K);


//        colorPin0 = hMap.digitalChannel.get("digital0");
//        colorPin1 = hMap.digitalChannel.get("digital1");

        intakePivot.setDirection(Servo.Direction.REVERSE);
        intakeRightSlide.setDirection(Servo.Direction.REVERSE);
        intakeLeftSlide.setDirection(Servo.Direction.FORWARD);
        turretServo.setDirection(Servo.Direction.FORWARD);

        LimelightDetection.mode = 2;
        LimelightDetection.secondMode = -1;
        resetArmForIntake();
        intakeSlidesIn();

    }

    @Override
    public void periodic() {
        if (scanningEnabled) {
            // run Limelight each tick
            limelight.runDetection();

            if (LimelightDetection.resultExists) {
                // store the last valid sample
                lastTarget = LimelightDetection.worldCoordinates;
                lastYaw    = LimelightDetection.sampleYaw;
                telemetry.addData("Limelight stored sample",
                        String.format("(%.2f, %.2f), yaw=%.4f",
                                lastTarget.x, lastTarget.y, lastYaw));
            } else {
                telemetry.addData("Limelight", "No valid target this tick");
            }
        } else {
            telemetry.addLine("Scanning disabled (arm is down)");
        }
        telemetry.update();
    }

    /**
     * “Fire” button: freeze the last sample and run a one-shot extend + pivot-down.
     * After this, scanningEnabled=false so no further sampling until resetArm().
     */
    public void fireOneShot() {
        if (lastTarget != null) {
            scanningEnabled = false;
            telemetry.addLine("fireOneShot: using last sample → aiming & pivot-down");
            telemetry.addData("lastTarget (x,y)", String.format("(%.2f, %.2f)",
                    lastTarget.x, lastTarget.y));
            telemetry.addData("lastYaw", String.format("%.4f", lastYaw));

            // run AutoAim on the frozen sample
            AutoAim.targetSystemPosition = lastTarget;
            AutoAim.sampleYaw            = lastYaw;
            AutoAim.setTargetPositions();

            // drop pivots
            intakePivot .setPosition(intakePivotDownPosition);
            telemetry.addData("Pivots", String.format("Dropped to (%.2f, %.2f)",
                    intakePivotDownPosition,
                    intakePivotDownPosition + 0.05));
        } else {
            telemetry.addData("fireOneShot", "No prior sample → nothing to aim");
        }
        telemetry.update();
    }

    /**
     * “Reset” button: raise pivots, retract slides, reset turret to 0.5, and re-enable scanning.
     */
    public void resetArmForIntake() {
        scanningEnabled = true;
        IntakePivotPos();
        turretReset();
        intakeSlidesIn();
        intakeClawOpen();

        telemetry.addLine("resetArm: pivots up, slides in, turret=0.5, resuming scanning");
        telemetry.update();
    }

    public void resetArmForTransfer() {
        scanningEnabled = false;
        intakeClawClose();
        intakePivotUp();
        turretResetTransfer();
        intakeSlidesIn();

        telemetry.addLine("resetArm: pivots up, slides in, turret=0.5, resuming scanning");
        telemetry.update();
    }

    public void turretReset(){
        turretServo.setPosition(turretRest);
    }
    public void IntakePivotPos(){
        intakePivot.setPosition(intakeIntakePos);
    }

    public void turretResetTransfer(){
        turretServo.setPosition(turretTransfer);
    }

    public void turretSideTransfer(){
        turretServo.setPosition(turretSide);
    }

//    public void Intake() {
//        //Turns the intake on
//        intakeMotor.setPower(1.0);
//    }
//
//    public void IntakeOff(){
//        intakeMotor.setPower(0);
//    }
//
//    public void Outtake() {
//        //Revers the intake
//        intakeMotor.setPower(-1);
//    }
//
//    public void slowIntake(){
//        intakeMotor.setPower(0.3);
//    }

    public void intakeSlidesIn() {
        //Brings the slides in
        intakeLeftSlide.setPosition(intakeSlidesInPosition);
        intakeRightSlide.setPosition(intakeSlidesInPosition);
    }

    public void IncrSlides(){
        double currentPos = getIntakeSlidePosition();
        //slides must be decremented
        double newPos = currentPos - 0.01;

        if(newPos < intakeSlidesOutPosition){
            newPos = intakeSlidesOutPosition;
        }
        setIntakeSlidePosition(newPos);
    }

    public void IncrSlidesFaster(){
        double currentPos = getIntakeSlidePosition();
        //slides must be decremented
        double newPos = currentPos - 0.05;

        if(newPos < intakeSlidesOutPosition){
            newPos = intakeSlidesOutPosition;
        }
        setIntakeSlidePosition(newPos);
    }

//    public boolean hasItemInIntake(){
//        telemetry.addData("IntakeColour", getDesiredIntakeColour());
//        telemetry.addData("Desired", getDesiredIntakeColour());
//        telemetry.addData("Result", getCurrentIntakeColour() == getDesiredIntakeColour());
//        telemetry.update();
//
//        return getCurrentIntakeColour() == getDesiredIntakeColour();
//
//    }

    public boolean AreIntakeSlidesIn() {
        return true;
    }

    public void intakeSlidesOut() {
        //Brings the slides out
        intakeLeftSlide.setPosition(intakeSlidesOutPosition);
        intakeRightSlide.setPosition(intakeSlidesOutPosition);


    }

    public void intakeSlidesHalfOut(){
        intakeLeftSlide.setPosition(intakeSlidesOutPosition * 2.5);
        intakeRightSlide.setPosition(intakeSlidesOutPosition * 2.5);
    }

//    public void SetPoopMode(boolean mode){
//        isPooping = mode;
//    }
//
//    public boolean IsPooping(){
//        return isPooping;
//    }

    public double getIntakeSlidePosition(){
        return intakeLeftSlide.getPosition();
    }

    public void setIntakeSlidePosition(double amount) {
        intakeLeftSlide.setPosition(amount);
        intakeRightSlide.setPosition(amount);
    }

    public boolean AreIntakeSlidesOut() {
        return true;
    }
    public void intakeClawYawBase(){
        clawYawServo.setPosition(0.5);
    }

    public void intakeClawYawSecond(){
        clawYawServo.setPosition(0);
    }

    public void intakePivotUp() {
        intakePivot.setPosition(intakePivotUpPosition);
    }

    public boolean IsIntakePivotedUp() {
        return true;
    }

    public void intakePivotDown() {
        intakePivot.setPosition(intakePivotDownPosition);
    }

    public void intakeClawOpen(){
        intakeClaw.setPosition(0.0);
    }

    public void intakeClawClose(){
        intakeClaw.setPosition(0.5);
    }

    public void intakeClawLoose(){
        intakeClaw.setPosition(0.9);
    }

    public void colorNeutral(){
        LimelightDetection.mode = 2;
        LimelightDetection.secondMode = -1;
    }

    public void colorBlue(){
        LimelightDetection.mode = 1;
        LimelightDetection.secondMode = -1;
    }

    public void colorRed(){
        LimelightDetection.mode = 0;
        LimelightDetection.secondMode = -1;
    }

    public void colorRedOrNeutral(){
        LimelightDetection.mode = 0;
        LimelightDetection.secondMode = 1;
    }

    public void colorBlueOrNeutral(){
        LimelightDetection.mode = 1;
        LimelightDetection.secondMode = 1;
    }

    public boolean IsIntakePivotedDown() {
        return true;
    }

//    public void poopChuteOpen() {
//        poopChute.setPosition(intakePoopOpen);
//    }

    public boolean IsPoopChuteOpened(){
        return true;
    }



//    public void poopChuteClose() {
//        poopChute.setPosition(intakePoopClose);
//    }
//
//    public boolean IsPoopChuteClosed(){
//        return true;
//    }

//    public SampleColour getCurrentIntakeColour(){
//
//
//       NormalizedRGBA colors = colourSensor.getNormalizedColors();
//        Color.colorToHSV(colors.toColor(), hsvValues);
//
//        telemetry.addData("HSV", hsvValues[0]);
//        telemetry.addData("HSV2", hsvValues[1]);
//        telemetry.update();
//
//        if(hsvValues[0] > 200 ) {
//            if(desiredColour == SampleColour.BLUE_OR_NEUTRAL){
//                return SampleColour.BLUE_OR_NEUTRAL;
//            }
//            if(desiredColour == SampleColour.AUTO_ANY){
//                return SampleColour.AUTO_ANY;
//            }
//            return SampleColour.BLUE;
//        }
//        if(hsvValues[0] >= 45 && hsvValues[0] <=90) {
//            if(desiredColour == SampleColour.RED_OR_NEUTRAL){
//                return SampleColour.RED_OR_NEUTRAL;
//            }
//            if(desiredColour == SampleColour.BLUE_OR_NEUTRAL){
//                return SampleColour.BLUE_OR_NEUTRAL;
//            }
//            if(desiredColour == SampleColour.AUTO_ANY){
//                return SampleColour.AUTO_ANY;
//            }
//            //telemetry.addData("FOUND", "NEUTRAL");
//            //telemetry.update();
//            return SampleColour.NEUTRAL;
//        }
//        if(hsvValues[0] >= 0 && hsvValues[1] > 0) {
//            if(desiredColour == SampleColour.RED_OR_NEUTRAL){
//                return SampleColour.RED_OR_NEUTRAL;
//            }
//            if(desiredColour == SampleColour.AUTO_ANY){
//                return SampleColour.AUTO_ANY;
//            }
//            return SampleColour.RED;
//        }
//
//        return SampleColour.NONE;
//    }

    public void setDesiredColourBlue() {
        limelight.secondMode = -1;
        limelight.mode = 1;
    }

    public void setDesiredColourBlueOrNeutral() {
        limelight.secondMode = 1;
        limelight.mode = 1;
    }

    public void setDesiredColourRedOrNeutral() {
        limelight.secondMode = 1;
        limelight.mode = 0;
    }

    public boolean IsDesiredColourBlueSet() {
        return true;
    }

//    public void setDesiredColour(SampleColour colour){
//        desiredColour = colour;
//    }
    public void setDesiredColourRed() {

        limelight.secondMode = -1;
        limelight.mode = 0;
    }

    public boolean IsDesiredColourRedSet() {
        return true;
    }

    public void setDesiredColourNeutral() {
        limelight.secondMode = -1;
        limelight.mode = 2;
    }

//    public boolean IsDesiredColourNeutralSet() {
//        return true;
//    }
//
//    public SampleColour getDesiredIntakeColour(){
//        return  desiredColour;
//    }


//    public void colourAwareIntake(){
//
//            //telemetry.addData("Desired:", desiredColour);
//            //telemetry.update();
//
//            SampleColour currentColour = getCurrentIntakeColour();
//
//            if (currentColour == SampleColour.NONE) {
//                poopChuteOpen();
//                this.Intake();
//            }
//            else if(desiredColour == SampleColour.BLUE_OR_NEUTRAL && (currentColour == SampleColour.BLUE || currentColour == SampleColour.NEUTRAL)){
//                this.IntakeOff();
//            }
//            else if(desiredColour == SampleColour.AUTO_ANY && (currentColour == SampleColour.AUTO_ANY)){
//                this.IntakeOff();
//            }
//            else if(desiredColour == SampleColour.RED_OR_NEUTRAL && (currentColour == SampleColour.RED || currentColour == SampleColour.NEUTRAL)){
//                this.IntakeOff();
//            }
//            else if(getCurrentIntakeColour() != desiredColour){
//                    if(!IsPooping()) {
//                        this.Outtake();
//                    }
//
//            }else {
//
//                this.IntakeOff();
//            }
    }

