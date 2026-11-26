
package org.firstinspires.ftc.teamcode.Senior;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class WarehouseVisionProcessor implements VisionProcessor {

    private final Mat hsv = new Mat();
    private final Mat greenMask = new Mat();
    private final Mat gray = new Mat();
    private final Mat binary = new Mat();

    private final Mat labels = new Mat();
    private final Mat stats = new Mat();
    private final Mat centroids = new Mat();

    private volatile boolean boxPresent = false;
    private volatile int greenPixels = 0;
    private volatile int dotCardNumber = 0;

    private int positiveFrames = 0;

    // Adjust after viewing camera telemetry.
    private static final int GREEN_THRESHOLD = 600;

    @Override
    public void init(
            int width,
            int height,
            CameraCalibration calibration) {
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

        if (frame.empty()) {
            return null;
        }

        int x = frame.cols() / 4;
        int y = frame.rows() / 5;

        int w = frame.cols() / 2;
        int h = (frame.rows() * 3) / 5;

        Rect roi = new Rect(x, y, w, h);

        // -------------------------------
        // 1. GREEN BOX DETECTION
        // -------------------------------

        Imgproc.cvtColor(
                frame,
                hsv,
                Imgproc.COLOR_RGB2HSV
        );

        Core.inRange(
                hsv,
                new Scalar(35, 70, 45),
                new Scalar(95, 255, 255),
                greenMask
        );

        Mat greenRegion = greenMask.submat(roi);

        greenPixels = Core.countNonZero(greenRegion);

        greenRegion.release();

        // Require multiple positive frames
        // to reduce false detections.

        if (greenPixels > GREEN_THRESHOLD) {
            positiveFrames = Math.min(8, positiveFrames + 1);
        } else {
            positiveFrames = Math.max(0, positiveFrames - 1);
        }

        boxPresent = positiveFrames >= 4;

        // -------------------------------
        // 2. DOT CARD RECOGNITION
        // -------------------------------

        Imgproc.cvtColor(
                frame,
                gray,
                Imgproc.COLOR_RGB2GRAY
        );

        Mat cardRegion = gray.submat(roi);

        Imgproc.threshold(
                cardRegion,
                binary,
                70,
                255,
                Imgproc.THRESH_BINARY_INV
        );

        cardRegion.release();

        int components = Imgproc.connectedComponentsWithStats(
                binary,
                labels,
                stats,
                centroids,
                8,
                CvType.CV_32S
        );

        int detectedDots = 0;

        for (int i = 1; i < components; i++) {

            int area = (int) stats.get(
                    i, Imgproc.CC_STAT_AREA
            )[0];

            int width = (int) stats.get(
                    i, Imgproc.CC_STAT_WIDTH
            )[0];

            int height = (int) stats.get(
                    i, Imgproc.CC_STAT_HEIGHT
            )[0];

            // Ignore very small noise and large objects.
            if (area >= 8 && area <= 1200 &&
                    width >= 2 && height >= 2) {

                double ratio = (double) width / height;

                if (ratio >= 0.5 && ratio <= 1.8) {
                    detectedDots++;
                }
            }
        }

        dotCardNumber =
                detectedDots >= 1 && detectedDots <= 9
                        ? detectedDots : 0;

        return null;
    }

    public boolean isBoxPresent() {
        return boxPresent;
    }

    public int getGreenPixels() {
        return greenPixels;
    }

    public int getDotCardNumber() {
        return dotCardNumber;
    }

    @Override
    public void onDrawFrame(
            Canvas canvas,
            int onscreenWidth,
            int onscreenHeight,
            float scaleBmpPxToCanvasPx,
            float scaleCanvasDensity,
            Object userContext) {
    }

    public void close() {
        hsv.release();
        greenMask.release();
        gray.release();
        binary.release();
        labels.release();
        stats.release();
        centroids.release();
    }
}
