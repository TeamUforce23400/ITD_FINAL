package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import org.opencv.core.Point3;

@Config
public final class AutoAim {

    // ─── Tuning Parameters ───────────────────────────────────────────────
    public static double ARM_LENGTH_CM      = 2.4;       // measured arm length (cm)

    public static double BASE_TURRET_POS    = 0.59;      // turret straight servo position
    public static double TURRET_POS_RANGE   = 0.24;      // servo range for full swing
    public static double TURRET_ANG_RANGE   = Math.toRadians(75);

    public static double BASE_CLAW_YAW      = 0;
    public static double CLAW_POS_RANGE     = 0.5;
    public static double CLAW_ANG_RANGE     = Math.toRadians(90);

    public static double BASE_EXT_POS       = 0.65;      // slides retracted servo pos
    public static double EXT_POS_RANGE      = -0.59;     // delta to full extend
    public static double MAX_EXT_CM         = 44;      // max slide travel (cm)

    public static double TURRET_DEADZONE_CM = 0.0;       // react to any lateral offset

    // ─── Runtime Inputs ───────────────────────────────────────────────────
    public static Point3 targetSystemPosition;
    public static double sampleYaw;

    // ─── Internal state for change thresholds ─────────────────────────────
    private static double lastTurretPos = -1;
    private static double lastExtPos    = -1;
    private static double lastClawPos   = -1;
    private static final double THRESHOLD = 0.02;

    /** Call this after setting targetSystemPosition & sampleYaw */
    public static void setTargetPositions() {
        if (targetSystemPosition != null) {
            extendTo(
                    targetSystemPosition.x,
                    targetSystemPosition.y,
                    sampleYaw
            );
        }
    }

    public static void extendTo(double x, double y, double h) {
        // ─── 1. Turret Rotation ──────────────────────────────────────────
        double tAng = 0;
        if (Math.abs(x) > TURRET_DEADZONE_CM) {
            double safeX = Math.min(Math.abs(x), ARM_LENGTH_CM);
            tAng = Math.signum(x) * Math.asin(safeX / ARM_LENGTH_CM);
        }
        double tDelta    = tAng * TURRET_POS_RANGE / TURRET_ANG_RANGE;
        double turretPos = clampServo(BASE_TURRET_POS + tDelta);
        if (Math.abs(turretPos - lastTurretPos) > THRESHOLD) {
            IntakeSubsystem.turretServo.setPosition(turretPos);
            lastTurretPos = turretPos;
        }

        // ─── 2. Slide Extension using Euclidean distance ────────────────
        double distance = Math.hypot(x, y);
        // Use raw distance so slides move even for close targets
        double required = distance;
        double frac     = Math.min(required / MAX_EXT_CM, 1.0);
        double eDelta   = frac * EXT_POS_RANGE;
        double ePos     = clampServo(BASE_EXT_POS + eDelta);
        // Always update slides (no deadband) to reflect target movement
        IntakeSubsystem.intakeLeftSlide.setPosition(ePos);
        IntakeSubsystem.intakeRightSlide.setPosition(ePos);
        lastExtPos = ePos;

        // ─── 3. Claw Yaw ──────────────────────────────────────────────────
        double cAng = normalize(
                Math.PI/2
                        + normalize(h + Math.PI/2)
                        - (tAng - Math.signum(tAng)*Math.PI/2)
        );
        double cDelta = cAng * CLAW_POS_RANGE / CLAW_ANG_RANGE;
        double cPos   = clampServo(BASE_CLAW_YAW - cDelta);
        if (Math.abs(cPos - lastClawPos) > THRESHOLD) {
            IntakeSubsystem.clawYawServo.setPosition(cPos);
            lastClawPos = cPos;
        }
    }

    /** Normalize any angle into [–π/2 … +π/2] **/
    private static double normalize(double ang) {
        ang = (ang + Math.PI/2) % Math.PI;
        if (ang < 0) ang += Math.PI;
        return ang - Math.PI/2;
    }

    /** Clamp servo values to [0, 1] **/
    private static double clampServo(double pos) {
        return Math.max(0.0, Math.min(1.0, pos));
    }

    /** Reset internal servo-state tracking after manual resets */
    public static void resetLastPositions() {
        lastTurretPos = IntakeSubsystem.turretServo.getPosition();
        lastExtPos    = IntakeSubsystem.intakeLeftSlide.getPosition();
        lastClawPos   = IntakeSubsystem.clawYawServo.getPosition();
    }
}