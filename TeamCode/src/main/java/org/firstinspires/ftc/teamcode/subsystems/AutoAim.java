package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import org.opencv.core.Point3;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * AutoAim helper drives three servos:
 *  • turretServo  (left/right)
 *  • clawYawServo (twist)
 *  • intakeLeftSlide/intakeRightSlide (horizontal extension)
 *
 * Every intermediate/calculated value is sent to telemetry.
 */
@Config
public final class AutoAim {
    // ─── TUNABLE PARAMETERS ───────────────────────────────────────────────────────
    public static double ARM_LENGTH_CM    = 28.3;

    public static double BASE_TURRET_POS  = 0.5;
    public static double TURRET_POS_RANGE = 0.5;
    public static double TURRET_ANG_RANGE = Math.toRadians(90);

    public static double BASE_CLAW_YAW    = 0.5;
    public static double CLAW_POS_RANGE   = 0.4;
    public static double CLAW_ANG_RANGE   = Math.toRadians(90);

    // for servo-based extension:
    public static double BASE_EXT_POS     = 0.5;   // midpoint "retracted"
    public static double EXT_POS_RANGE    = 0.4;   // full Δ for max reach
    public static double MAX_EXT_CM       = 30.0;  // how far (cm) slide can extend

    // ─── RUNTIME FIELDS ─────────────────────────────────────────────────────────
    public static Point3 targetSystemPosition;
    public static double  sampleYaw;

    /** Telemetry reference (must be set once by IntakeSubsystem) */
    private static Telemetry telemetry = null;

    /**
     * Call this early (e.g. from IntakeSubsystem constructor) so that AutoAim
     * can report its values via telemetry.
     */
    public static void setTelemetry(Telemetry t) {
        telemetry = t;
        telemetry.addData("AutoAim", "Telemetry hooked");
        telemetry.update();
    }

    /**
     * Call after setting targetSystemPosition & sampleYaw. Internally calls extendTo(...).
     */
    public static void setTargetPositions() {
        if (telemetry == null) {
            throw new IllegalStateException("AutoAim.telemetry must be set before calling setTargetPositions()");
        }
        if (targetSystemPosition != null) {
            extendTo(
                    targetSystemPosition.x,
                    targetSystemPosition.y,
                    sampleYaw
            );
        } else {
            telemetry.addData("AutoAim", "No targetSystemPosition set → skipping extendTo()");
            telemetry.update();
        }
    }

    /**
     * Main aiming math → drives IntakeSubsystem servos.
     * Reports every intermediate/calculated value to telemetry.
     *
     * @param x lateral offset (cm)
     * @param y forward  offset (cm)
     * @param h sampled yaw (0 or π/2)
     */
    public static void extendTo(double x, double y, double h) {
        telemetry.addLine("---- AutoAim extendTo start ----");
        telemetry.addData("Input (x,y,h)", String.format("(%.2f, %.2f, %.4f)", x, y, h));

        // 1) turret rotation angle (rad)
        double tAng = Math.signum(x) * Math.asin(Math.abs(x) / ARM_LENGTH_CM);
        double tDelta = tAng * TURRET_POS_RANGE / TURRET_ANG_RANGE;
        telemetry.addData("turretAngle (rad)", String.format("%.4f", tAng));
        telemetry.addData("turretDelta (servo)", String.format("%.4f", tDelta));

        // 2) extension reach
        double reachCm = y - ARM_LENGTH_CM * Math.cos(tAng);
        double frac = Math.max(0, Math.min(reachCm / MAX_EXT_CM, 1.0));
        double eDelta = frac * EXT_POS_RANGE;
        double ePos = BASE_EXT_POS + eDelta;
        telemetry.addData("reachCm", String.format("%.2f", reachCm));
        telemetry.addData("clamped frac (0→1)", String.format("%.4f", frac));
        telemetry.addData("eDelta (servo)", String.format("%.4f", eDelta));
        telemetry.addData("extensionPos", String.format("%.4f", ePos));

        // 3) claw yaw angle (rad) to keep jaws level
        double cAng = normalize(
                Math.PI/2
                        + normalize(h + Math.PI/2)
                        - (tAng - Math.signum(tAng) * Math.PI/2)
        );
        double cDelta = cAng * CLAW_POS_RANGE / CLAW_ANG_RANGE;
        telemetry.addData("clawAngle (rad)", String.format("%.4f", cAng));
        telemetry.addData("clawDelta (servo)", String.format("%.4f", cDelta));

        // 4) commit to hardware (via IntakeSubsystem’s static references)
        double turretServoPos    = BASE_TURRET_POS + tDelta;
        double clawYawServoPos   = BASE_CLAW_YAW - cDelta;
        double leftSlideServoPos = ePos;
        double rightSlideServoPos= ePos;

        IntakeSubsystem.turretServo.setPosition(turretServoPos);
        IntakeSubsystem.clawYawServo.setPosition(clawYawServoPos);
        IntakeSubsystem.intakeLeftSlide.setPosition(leftSlideServoPos);
        IntakeSubsystem.intakeRightSlide.setPosition(rightSlideServoPos);

        telemetry.addData("→ turretServo pos", String.format("%.4f", turretServoPos));
        telemetry.addData("→ clawYawServo pos", String.format("%.4f", clawYawServoPos));
        telemetry.addData("→ intakeLeftSlide pos", String.format("%.4f", leftSlideServoPos));
        telemetry.addData("→ intakeRightSlide pos", String.format("%.4f", rightSlideServoPos));

        telemetry.addLine("---- AutoAim extendTo end ----");
        telemetry.update();
    }

    /** Normalize any angle into [–π/2 … +π/2] **/
    private static double normalize(double ang) {
        ang = (ang + Math.PI/2) % Math.PI;
        if (ang < 0) ang += Math.PI;
        return ang - Math.PI/2;
    }
}
