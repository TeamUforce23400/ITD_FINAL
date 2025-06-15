package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import org.opencv.core.Point3;

@Config
public final class AutoAim {

    // ─── Tuning Parameters ───────────────────────────────────────────────
    public static double ARM_LENGTH_CM      = 13.5;

    public static double BASE_TURRET_POS    = 0.59;
    public static double TURRET_POS_RANGE   = 0.31;
    public static double TURRET_ANG_RANGE   = Math.toRadians(90);

    public static double BASE_CLAW_YAW      = 0.5;
    public static double CLAW_POS_RANGE     = -0.5;
    public static double CLAW_ANG_RANGE     = Math.toRadians(90);

    public static double BASE_EXT_POS       = 0.65;   // Retracted
    public static double EXT_POS_RANGE      = -0.59;  // Fully extended → 0.06
    public static double MAX_EXT_CM         = 48.0;

    public static double TURRET_DEADZONE_CM = 0.5;    // Ignore small turret changes

    // ─── Runtime Inputs ───────────────────────────────────────────────────
    public static Point3 targetSystemPosition;
    public static double sampleYaw;

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
            double xClamped = Math.min(Math.abs(x) / ARM_LENGTH_CM, 1.0);
            tAng = Math.signum(x) * Math.asin(xClamped);
        }
        double tDelta = tAng * TURRET_POS_RANGE / TURRET_ANG_RANGE;
        double turretPos = clampServo(BASE_TURRET_POS + tDelta);

        // ─── 2. Slide Extension (with reach clamped) ─────────────────────
        double reach = y - ARM_LENGTH_CM * Math.cos(tAng);
        reach = Math.max(5.0, reach);  // Prevent weak/no extension
        double frac = Math.min(reach / MAX_EXT_CM, 1.0);
        double eDelta = frac * EXT_POS_RANGE;
        double ePos = clampServo(BASE_EXT_POS + eDelta);

        // ─── 3. Claw Yaw ──────────────────────────────────────────────────
        double cAng = normalize(
                Math.PI / 2
                        + normalize(h + Math.PI / 2)
                        - (tAng - Math.signum(tAng) * Math.PI / 2)
        );
        double cDelta = cAng * CLAW_POS_RANGE / CLAW_ANG_RANGE;
        double cPos = clampServo(BASE_CLAW_YAW - cDelta);

        // ─── 4. Apply to Servos ───────────────────────────────────────────
        IntakeSubsystem.turretServo.setPosition(turretPos);
        IntakeSubsystem.clawYawServo.setPosition(cPos);
        IntakeSubsystem.intakeLeftSlide.setPosition(ePos);
        IntakeSubsystem.intakeRightSlide.setPosition(ePos);

        // ─── 5. Debug Logging ─────────────────────────────────────────────
//        System.out.printf(
//                "AutoAim:\n  x=%.2f, y=%.2f, h=%.2f\n  tAng=%.4f rad, tDelta=%.4f → turret=%.3f\n  reach=%.2f → ePos=%.3f\n  cAng=%.4f → cPos=%.3f\n",
//                x, y, h, tAng, tDelta, turretPos, reach, ePos, cAng, cPos
//        );
    }

    /** Normalize any angle into [–π/2 … +π/2] **/
    private static double normalize(double ang) {
        ang = (ang + Math.PI / 2) % Math.PI;
        if (ang < 0) ang += Math.PI;
        return ang - Math.PI / 2;
    }

    /** Clamp servo values to [0, 1] **/
    private static double clampServo(double pos) {
        return Math.max(0.0, Math.min(1.0, pos));
    }
}
