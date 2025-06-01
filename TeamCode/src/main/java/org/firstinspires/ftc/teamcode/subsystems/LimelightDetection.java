package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Point;
import org.opencv.core.Point3;

import java.util.LinkedList;
import java.util.List;

@Config
public class LimelightDetection {

    public static Limelight3A limelight;
    private final Telemetry telemetry;

    public LimelightDetection(HardwareMap hardwareMap, Telemetry tele) {
        this.telemetry = tele;

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);            // Make sure this matches your Dashboard’s pipeline
        limelight.setPollRateHz(400);
        limelight.start();

        telemetry.addData("LimelightDetection", "Initialized (mode=%d, secondMode=%d)", mode, secondMode);
        telemetry.update();
    }

    /** External parameters (adjust via Dashboard if needed) **/
    public static int mode = 2;
    public static int secondMode = -1;
    final double X_CAMERA = 0;
    final double Y_CAMERA = 10;
    final double Z_CAMERA = 20;
    public static double PITCH_CAMERA = Math.toRadians(10);

    /** Auxiliary variables **/
    public ElapsedTime detectionTimer = new ElapsedTime();
    public static int detectionCounter = 0;

    private final int MAX_FRAMES = 5;
    private final LinkedList<Point3> detectionBuffer = new LinkedList<>();

    public static Point3 faraMedieWorldCoordinates;
    public static Point3 worldCoordinates;
    public static double sampleYaw;
    public static double aspectRatio = 0;

    public static List<List<Double>> vertices;
    public Point[] points = new Point[4];

    public static int targetIndex = -1;
    public static double horizontalAngle, verticalAngle;
    public static double detectionPosX, detectionPosY;

    public static boolean resultExists = false;

    public static void resetVariables() {
        targetIndex  = -1;
        resultExists = false;
    }

    public boolean isGoodColor(LLResultTypes.DetectorResult target) {
        return (target.getClassId() == mode || target.getClassId() == secondMode);
    }

    public boolean isWithinRange(double tx, double ty) {
        return ty < 15 && ty > -10 && Math.abs(tx) <= 14;
    }

    public boolean isCloser(double tx, double ty, double minTx, double minTy) {
        if (ty < minTy) return true;
        return (minTy == ty && tx < minTx);
    }

    /**
     * This method:
     *  1) Calls getLatestResult()
     *  2) Logs every blob’s (classID, tx, ty)
     *  3) Logs each blob’s pass/fail of isGoodColor and isWithinRange
     *  4) Logs minTx/minTy and isCloser decisions
     *  5) After the loop, logs final resultExists and targetIndex
     *  6) If resultExists, writes out corner coordinates and then calls telemetry.update()
     *  7) If no valid target, does not call telemetry.update()
     */
    public void getResult() {
        // 1) Fetch latest Limelight result
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            // skip all telemetry if no valid frame
            return;
        }

        // 2) List all detected blobs BEFORE filtering
        List<LLResultTypes.DetectorResult> detectedTargets = result.getDetectorResults();
//        telemetry.addData("Detected blobs (count)", detectedTargets.size());
        for (int i = 0; i < detectedTargets.size(); i++) {
            LLResultTypes.DetectorResult t = detectedTargets.get(i);
            double tx = t.getTargetXDegrees();
            double ty = t.getTargetYDegrees();
            int cID = t.getClassId();
//            telemetry.addData(
//                    String.format("Blob %d (class, tx, ty)", i),
//                    String.format("(%d, %.2f, %.2f)", cID, tx, ty)
//            );
        }

        // 3) Now run your filtering logic to pick the “closest” valid blob
        double minTx = Double.POSITIVE_INFINITY;
        double minTy = Double.POSITIVE_INFINITY;
        targetIndex  = -1;
        resultExists = false;

        for (int i = 0; i < detectedTargets.size(); i++) {
            LLResultTypes.DetectorResult target = detectedTargets.get(i);
            double ty = target.getTargetYDegrees();
            double tx = target.getTargetXDegrees();
            int cID = target.getClassId();

            boolean goodColor   = isGoodColor(target);
            boolean withinRange = isWithinRange(tx, ty);
//            telemetry.addData(
//                    String.format("Blob %d checks", i),
//                    String.format("colorOK=%b, rangeOK=%b", goodColor, withinRange)
//            );

            if (goodColor && withinRange) {
                boolean closer = isCloser(tx, ty, minTx, minTy);
//                telemetry.addData(
//                        String.format("→ isCloser check for blob %d", i),
//                        String.format("ty(%.2f) < minTy(%.2f)? %b ; or equal-ty & tx(%.2f) < minTx(%.2f)? %b",
//                                ty, minTy, (ty < minTy), tx, minTx, (minTy == ty && tx < minTx))
//                );
                if (closer) {
                    minTx = Math.abs(tx);
                    minTy = ty;
                    horizontalAngle = -Math.toRadians(tx);
                    verticalAngle   = Math.toRadians(ty);
                    targetIndex     = i;
                    resultExists    = true;

//                    telemetry.addData("→ Chosen Index", i);
//                    telemetry.addData("→ Chosen (tx, ty)", String.format("(%.2f, %.2f)", tx, ty));
//                    telemetry.addData("→ hAngle (rad)", String.format("%.4f", horizontalAngle));
//                    telemetry.addData("→ vAngle (rad)", String.format("%.4f", verticalAngle));
                }
            }
        }

        // 4) After loop, report whether we found any valid target
        telemetry.addData("After loop: resultExists", resultExists);
        telemetry.addData("After loop: targetIndex", targetIndex);

        // 5) If found, extract that blob’s corners into points[] and then update telemetry
        if (resultExists) {
            vertices = detectedTargets.get(targetIndex).getTargetCorners();
            telemetry.addData("Corners count", vertices.size());
            for (int i = 0; i < vertices.size() && i < points.length; i++) {
                points[i] = new Point(vertices.get(i).get(0), vertices.get(i).get(1));
                telemetry.addData(
                        String.format("Corner %d (px, py)", i),
                        String.format("(%.1f, %.1f)", points[i].x, points[i].y)
                );
                if (i < 3) {
                    telemetry.addData(
                            String.format("points[%d].x, .y", i),
                            String.format("(%.1f, %.1f)", points[i].x, points[i].y)
                    );
                }
            }
            telemetry.update();
        }
        // If no resultExists, do NOT call telemetry.update()
    }

    private double euclideanDist(Point p1, Point p2) {
        double dx = p1.x - p2.x;
        double dy = p1.y - p2.y;
        double d = Math.hypot(dx, dy);
        if (p1.x == p2.x) {
            d *= 1.8; // fudge factor for perfect vertical edge
        }
        telemetry.addData("euclidDist", String.format("dx=%.1f, dy=%.1f, d=%.1f", dx, dy, d));
        return d;
    }

    public void getPositions() {
        double normalTargetAngle = PITCH_CAMERA + verticalAngle;
        double yDist = Z_CAMERA * Math.tan(normalTargetAngle);
        double cameraTargetDist = Math.hypot(Z_CAMERA, yDist);
        double xDist = cameraTargetDist * Math.tan(horizontalAngle);

//        telemetry.addData("normalAngle (rad)", String.format("%.4f", normalTargetAngle));
//        telemetry.addData("yDist (cm)",        String.format("%.2f", yDist));
//        telemetry.addData("camDist (cm)",      String.format("%.2f", cameraTargetDist));
//        telemetry.addData("xDist (cm)",        String.format("%.2f", xDist));

        double worldX = X_CAMERA + xDist;
        double worldY = Y_CAMERA + yDist;
        detectionPosX = worldX;
        detectionPosY = worldY;

        // Add to buffer
        detectionBuffer.add(new Point3(worldX, worldY, 0));
        if (detectionBuffer.size() > MAX_FRAMES) {
            detectionBuffer.removeFirst();
        }

        // Compute rolling average
        double sumX = 0, sumY = 0;
        for (Point3 p : detectionBuffer) {
            sumX += p.x;
            sumY += p.y;
        }

        worldCoordinates = new Point3(sumX / detectionBuffer.size(),
                sumY / detectionBuffer.size(),
                0);
        faraMedieWorldCoordinates = new Point3(worldX, worldY, 0);

//        telemetry.addData("latestWorld (x,y)", String.format("(%.2f, %.2f)", worldX, worldY));
//        telemetry.addData("avgWorld (x,y)",    String.format("(%.2f, %.2f)",
//                worldCoordinates.x,
//                worldCoordinates.y));
//        telemetry.update();
    }

    // The getYaw() method remains commented out as requested
//    public void getYaw() {
//        double length = euclideanDist(points[0], points[1]);
//        double width = euclideanDist(points[1], points[2]);
//
//        telemetry.addData("length vs width", String.format("%.1f vs %.1f", length, width));
//
//        if (length > width) {
//            sampleYaw = Math.PI / 2;
//        } else {
//            sampleYaw = 0;
//        }
//        telemetry.addData("sampleYaw (rad)", String.format("%.4f", sampleYaw));
//        telemetry.update();
//    }

    public void runDetection() {
        resetVariables();
        telemetry.addLine("---- runDetection start ----");
//        telemetry.update();

        getResult();

        if (resultExists) {
            telemetry.addData("Limelight", "Target found! (index=%d)", targetIndex);
            telemetry.update();
            getPositions();
//            getYaw();  // Still commented out if you prefer
        } else {
            telemetry.addData("Limelight", "No target found");
//            telemetry.update();
        }

        telemetry.addData("Limelight resultExists", resultExists);
        telemetry.addLine("---- runDetection end ----");
//        telemetry.update();
    }
}
