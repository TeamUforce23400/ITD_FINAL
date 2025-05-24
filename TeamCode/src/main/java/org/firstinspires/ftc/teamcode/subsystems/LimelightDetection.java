package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Point;
import org.opencv.core.Point3;

import java.util.LinkedList;
import java.util.List;

@Config
public class LimelightDetection{

    public static Limelight3A limelight;

    public LimelightDetection(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(2);
        limelight.setPollRateHz(400);
        limelight.start();
    }

    /**     External parameters     */
    public static int mode = 2;
    public static int secondMode = -1;
    final double X_CAMERA = 0;
    final double Y_CAMERA = 10;
    final double Z_CAMERA = 18;
    public static double PITCH_CAMERA = Math.toRadians(42);

    /**     Auxiliary variables     **/
    public ElapsedTime detectionTimer = new ElapsedTime();
    public static int detectionCounter = 0;

    private final int MAX_FRAMES = 5;
    private final LinkedList<Point3> detectionBuffer = new LinkedList<>();


    public static Point3 faraMedieWorldCoordinates;
    public static Point3 worldCoordinates;
    public static double sampleYaw;
    public static double aspectRatio = 0;

    public static List<List<Double>> vertices;
    public  Point[] points = new Point[4];

    public static int targetIndex = -1;
    public static double horizontalAngle, verticalAngle;
    public static double detectionPosX, detectionPosY;

    public static boolean resultExists = false;

    public static void resetVariables() {
        targetIndex = -1;
        resultExists= false;
    }

    public boolean isGoodColor(LLResultTypes.DetectorResult target) {
        return (target.getClassId() == mode || target.getClassId() == secondMode);
    }

    public boolean isWithinRange(double tx, double ty) {
        return ty < 15 && ty > -3 && Math.abs(tx) <= 14;
    }

    public boolean isCloser(double tx, double ty, double minTx, double minTy) {
        if (ty < minTy) return true;
        return minTy == ty && tx < minTx;
    }

    public void getResult() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.DetectorResult> detectedTargets = result.getDetectorResults();
            //
            double minTx = (1<<30), minTy = (1<<30);

            for (int i = 0; i < detectedTargets.size(); i++) {
                LLResultTypes.DetectorResult target = detectedTargets.get(i);

                double ty = target.getTargetYDegrees(),
                        tx = target.getTargetXDegrees();

                if (isGoodColor(target) && isWithinRange(tx, ty))
                    if (isCloser(tx, ty, minTx, minTy)) {
                        minTx = Math.abs(tx);
                        minTy = ty;

                        horizontalAngle = -Math.toRadians(tx);
                        verticalAngle = Math.toRadians(ty);
                        targetIndex = i;

                        resultExists = true;
                    }
            }
            //
            if (resultExists) {
                vertices = detectedTargets.get(targetIndex).getTargetCorners();

                for (int i = 0; i < vertices.size(); i++)
                    points[i] = new Point(vertices.get(i).get(0), vertices.get(i).get(1));
            }
        }
    }

    private double euclideanDist(Point p1, Point p2) {
        if (p1.x == p2.x)
            return  Math.sqrt(Math.pow(p1.x-p2.x, 2) + Math.pow(p1.y-p2.y, 2)) * 1.8;
        return  Math.sqrt(Math.pow(p1.x-p2.x, 2) + Math.pow(p1.y-p2.y, 2));
    }


    public void getPositions() {
        double normalTargetAngle = PITCH_CAMERA + verticalAngle;
        double yDist = Z_CAMERA * Math.tan(normalTargetAngle);
        double cameraTargetDist = Math.sqrt(Z_CAMERA * Z_CAMERA + yDist * yDist);
        double xDist = cameraTargetDist * Math.tan(horizontalAngle);

        // Compute current detection position
        double worldX = X_CAMERA + xDist;
        double worldY = Y_CAMERA + yDist;

        // Add to buffer
        detectionBuffer.add(new Point3(worldX, worldY, 0));
        if (detectionBuffer.size() > MAX_FRAMES) {
            detectionBuffer.removeFirst(); // Keep buffer size fixed
        }

        // Compute rolling average
        double sumX = 0;
        double sumY = 0;
        for (Point3 p : detectionBuffer) {
            sumX += p.x;
            sumY += p.y;
        }

        worldCoordinates = new Point3(sumX / detectionBuffer.size(), sumY / detectionBuffer.size(), 0);

        faraMedieWorldCoordinates = new Point3(worldX, worldY, 0);
    }

    public void getYaw() {
        double length = euclideanDist(points[0], points[1]),
                width = euclideanDist(points[1], points[2]);
        //
        if (length > width)
            sampleYaw = Math.PI / 2;
        else
            sampleYaw = 0;
    }

    public void runDetection() {
        resetVariables();
        getResult();

        if (resultExists) {
            getPositions();
            getYaw();
        }
    }
}