package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

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
    private double intakeSlidesInPosition = 0.655;

    private double intakeSlidesTransferPosition = 0.57;

    private double intakeSlidesOutPosition = 0.1;

    private double TurretPositionLeft = 0.83;
    private double TurretPositionRight = 0.34;

    private double intakePivotUpPosition = 0.0;
    private double intakeIntakePos = 0.45;
    private double intakePivotDownPosition = 1.0;

    private double intakePoopOpen = 0.85;

    private double intakePoopClose = 0.3;

    private double turretRest = 0.59;
    //0.59
    private double turretSide = 0.28;

    private double turretTransfer = 0.032;

    public static final double TURRET_LEFT_LIMIT  = 0.83;
    public static final double TURRET_RIGHT_LIMIT = 0.34;
    public static final double TURRET_STEP        = 0.02;
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
    private Point3 lastTarget = new Point3(0,0,0);

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
        AutoAim.resetLastPositions();
        intakeSlidesIn();

    }

    // 1) Add a separate flag for freezing the arm

    @Override
    public void periodic() {
        // Always run vision
        limelight.runDetection();

        // Always update the last valid sample
        if (LimelightDetection.resultExists) {
            lastTarget = LimelightDetection.worldCoordinates;
            lastYaw    = LimelightDetection.sampleYaw;
            telemetry.addData("Limelight stored", "(%.2f, %.2f), yaw=%.4f",
                    lastTarget.x, lastTarget.y, lastYaw);
        } else {
            lastTarget = new Point3(0,0,0);
            telemetry.addData("Limelight", "No valid target");
        }
        telemetry.update();
    }

    /**
     * Fire button: run AutoAim exactly once with the last sample.
     */
    public void fireOneShot() {
        if (lastTarget != null) {

            telemetry.addLine("fireOneShot: aiming & pivot-down");
            telemetry.addData("lastTarget", "(%.2f, %.2f)", lastTarget.x, lastTarget.y);
            telemetry.addData("lastYaw", "%.4f", lastYaw);

            // ① Auto-aim using that frozen sample
            AutoAim.targetSystemPosition = lastTarget;
            AutoAim.sampleYaw            = lastYaw;
            AutoAim.setTargetPositions();

            // ② Drop the intake pivot
            intakePivot.setPosition(intakeIntakePos);
        } else {
            telemetry.addData("fireOneShot", "No prior sample");
        }
        telemetry.update();
    }


    /**
     * Reset button: retract, reset turret, re-open claw, re-enable scanning.
     */
    public void resetArmForIntake() {
        // re-enable detection if you’d ever disabled it
        scanningEnabled = true;

        IntakePivotPos();
        turretReset();
        intakeSlidesIn();
        intakeClawOpen();

        AutoAim.resetLastPositions();

        telemetry.addLine("resetArm: slides in, turret reset, claw open");
        telemetry.update();
    }


    public void resetArmForTransfer() {
        scanningEnabled = false;
        intakeClawClose();
        turretResetTransfer();
        intakeSlidesTransfer();

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

    public void adjustTurret(double delta) {
        double pos = turretServo.getPosition() + delta;
        pos = Range.clip(pos, TURRET_RIGHT_LIMIT, TURRET_LEFT_LIMIT);
        turretServo.setPosition(pos);
    }

    public void IncrTurretLeft() {
        double currentPos = getTurretPosition();
        double newPos = currentPos + 0.02;
        // clamp *above* your left limit, not below
        if (newPos > TurretPositionLeft) {
            newPos = TurretPositionLeft;
        }
        setTurretPosition(newPos);

//        double p = 0.15;

    }

    public void IncrTurretRight() {
        double currentPos = getTurretPosition();
        double newPos = currentPos - 0.02;
        // clamp *below* your right limit, not above
        if (newPos < TurretPositionRight) {
            newPos = TurretPositionRight;
        }
        setTurretPosition(newPos);
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
        intakeLeftSlide.setPosition(0.3);
        intakeRightSlide.setPosition(0.3);
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

    public double getTurretPosition(){
        return turretServo.getPosition();
    }

    public void setIntakeSlidePosition(double amount) {
        intakeLeftSlide.setPosition(amount);
        intakeRightSlide.setPosition(amount);
    }

    public void setTurretPosition(double amount) {
        turretServo.setPosition(amount);
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

    public void intakeSlidesTransfer(){
        intakeRightSlide.setPosition(intakeSlidesTransferPosition);
        intakeLeftSlide.setPosition(intakeSlidesTransferPosition);
    }

    public void intakeSlidesFrontTransfer(){
        intakeRightSlide.setPosition(intakeSlidesTransferPosition-0.1);
        intakeLeftSlide.setPosition(intakeSlidesTransferPosition-0.1);
    }
    public void intakePivotUp() {
        intakePivot.setPosition(intakePivotUpPosition);
    }

    public void intakePivotMid() {
        intakePivot.setPosition(intakePivotUpPosition + 0.87);
    }

    public boolean IsIntakePivotedUp() {
        return true;
    }

    public void intakePivotDown() {
        intakePivot.setPosition(intakePivotDownPosition);
    }

    public void intakeClawOpen(){
        intakeClaw.setPosition(0.2);
    }

    public void intakeClawClose(){
        intakeClaw.setPosition(0.415);
    }

    public void intakeClawLoose(){
        intakeClaw.setPosition(0.39);
    }

    public void colorNeutral(){
        LimelightDetection.mode = 2;
        LimelightDetection.secondMode = -1;
    }

    public void colorBlue(){
        LimelightDetection.mode = 1;
        LimelightDetection.secondMode = 1;
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

//    public boolean IsPoopChuteOpened(){
//        return true;
//

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
//            if(desired

}
