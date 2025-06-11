package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import org.opencv.core.Point3;

@Config
public final class AutoAim {
    // ─── tuning parameters ───────────────────────────────────────────────────────
    public static double ARM_LENGTH_CM      = 28.3;

    public static double BASE_TURRET_POS    = 0.5;
    public static double TURRET_POS_RANGE   = 0.5;
    public static double TURRET_ANG_RANGE   = Math.toRadians(90);

    public static double BASE_CLAW_YAW      = 0.5;
    public static double CLAW_POS_RANGE     = 0.4;
    public static double CLAW_ANG_RANGE     = Math.toRadians(90);

    // for servo‐based extension:
    public static double BASE_EXT_POS       = 0.5;   // midpoint “retracted”
    public static double EXT_POS_RANGE      = 0.4;   // full Δ for max reach
    public static double MAX_EXT_CM         = 48.0;  // how far (cm) slide can extend

    // ─── runtime inputs ─────────────────────────────────────────────────────────
    public static Point3 targetSystemPosition;
    public static double  sampleYaw;

    /** call after setting targetSystemPosition & sampleYaw */
    public static void setTargetPositions() {
        if (targetSystemPosition != null) {
            extendTo(
                    targetSystemPosition.x,
                    targetSystemPosition.y,
                    sampleYaw
            );
        }
    }

    /**
     * Drives 3 servos in your IntakeSubsystem:
     *  • turretServo (left/right)
     *  • clawYawServo (twist)
     *  • extensionLeft/Right (slide)
     *
     * @param x lateral offset (cm)
     * @param y forward  offset (cm)
     * @param h sampled yaw (0 or π/2)
     */
    public static void extendTo(double x, double y, double h) {
        // 1) Turret rotation
        double tAng   = Math.signum(x) * Math.asin(Math.abs(x) / ARM_LENGTH_CM);
        double tDelta = tAng * TURRET_POS_RANGE / TURRET_ANG_RANGE;

        // 2) Horizontal extension
        double reach = y - ARM_LENGTH_CM * Math.cos(tAng);
        double frac  = Math.max(0, Math.min(reach / MAX_EXT_CM, 1.0));
        double eDelta= frac * EXT_POS_RANGE;
        double ePos  = BASE_EXT_POS + eDelta;

        // 3) Claw yaw to keep jaws level
        double cAng   = normalize(
                Math.PI/2
                        + normalize(h + Math.PI/2)
                        - (tAng - Math.signum(tAng) * Math.PI/2)
        );
        double cDelta = cAng * CLAW_POS_RANGE / CLAW_ANG_RANGE;

        // 4) Apply to servos
        IntakeSubsystem.turretServo.setPosition(BASE_TURRET_POS + tDelta);
        IntakeSubsystem.clawYawServo.setPosition(BASE_CLAW_YAW - cDelta);
        IntakeSubsystem.intakeLeftSlide.setPosition(ePos);
        IntakeSubsystem.intakeRightSlide.setPosition(ePos);
    }

    /** Normalize any angle into [–π/2 … +π/2] **/
    private static double normalize(double ang) {
        ang = (ang + Math.PI/2) % Math.PI;
        if (ang < 0) ang += Math.PI;
        return ang - Math.PI/2;
    }
}
