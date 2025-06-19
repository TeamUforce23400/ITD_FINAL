package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import org.opencv.core.Point3;

@Config
public final class AutoAim {

    // ─── Tuning Parameters ───────────────────────────────────────────────
    public static double ARM_LENGTH_CM      = 2.4;

    public static double BASE_TURRET_POS    = 0.59;
    public static double TURRET_POS_RANGE   = 0.24;
    public static double TURRET_ANG_RANGE   = Math.toRadians(75);

    public static double BASE_CLAW_YAW      = 0.5;
    public static double CLAW_POS_RANGE     = -0.5;
    public static double CLAW_ANG_RANGE     = Math.toRadians(90);

    public static double BASE_EXT_POS       = 0.65;
    public static double EXT_POS_RANGE      = -0.59;
    public static double MAX_EXT_CM         = 44;

    public static double TURRET_DEADZONE_CM = 0.0;

    public static double EXTENSION_BOOST    = 1.2; // ⬅️ Boost factor for compensation

    // ─── Runtime Inputs ───────────────────────────────────────────────────
    public static Point3 targetSystemPosition;
    public static double sampleYaw;

    // ─── Internal state ──────────────────────────────────────────────────
    private static double lastTurretPos = -1;
    private static double lastExtPos    = -1;
    private static double lastClawPos   = -1;
    private static final double THRESHOLD = 0.02;

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

        // ─── 2. Slide Extension with boosted turret compensation ─────────
        double distance = Math.hypot(x, y);
        double turretCompensation = Math.abs(Math.cos(tAng)) > 0.01
                ? EXTENSION_BOOST / Math.cos(tAng)
                : EXTENSION_BOOST;
        double required = distance * turretCompensation;

        double frac     = Math.min(required / MAX_EXT_CM, 1.0);
        double eDelta   = frac * EXT_POS_RANGE;
        double ePos     = clampServo(BASE_EXT_POS + eDelta);

        IntakeSubsystem.intakeLeftSlide.setPosition(ePos);
        IntakeSubsystem.intakeRightSlide.setPosition(ePos);
        lastExtPos = ePos;

        // ─── 3. Claw Yaw relative to turret ──────────────────────────────
        double relativeYaw = normalize(h - tAng);
        double cDelta = relativeYaw * CLAW_POS_RANGE / CLAW_ANG_RANGE;
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

    /** Reset servo tracking for manual overrides */
    public static void resetLastPositions() {
        lastTurretPos = IntakeSubsystem.turretServo.getPosition();
        lastExtPos    = IntakeSubsystem.intakeLeftSlide.getPosition();
        lastClawPos   = IntakeSubsystem.clawYawServo.getPosition();
    }
}
