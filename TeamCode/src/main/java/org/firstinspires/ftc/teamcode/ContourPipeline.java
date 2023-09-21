package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class ContourPipeline extends OpenCvPipeline {
    Scalar RED = new Scalar(196, 23, 112);

    // Defining static variables for lower and upper bounds of YCrCb color space
    public static Scalar scalarLowerYCrCb = new Scalar(0.0, 150.0, 120.0);
    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 255.0, 255.0);

    // Defining volatile variables to handle errors and debug information
    public volatile boolean error = false;
    public volatile Exception debug;

    // Defining variables to store the borders of the region of interest
    private double borderLeftX;
    private double borderRightX;
    private double borderTopY;
    private double borderBottomY;

    // Defining variables to store the camera dimensions
    private int CAMERA_WIDTH;
    private int CAMERA_HEIGHT;

    // Defining loop counters to keep track of the number of iterations
    private int loopCounter = 0;
    private int pLoopCounter = 0;

    // Defining Mats to store the original and processed images
    private final Mat mat = new Mat();
    private final Mat processed = new Mat();

    // Defining a Rect to store the maximum rectangle found in the contours
    private Rect maxRect = new Rect(600, 1, 1, 1);

    // Defining variables to store the maximum area found and a flag to indicate the first iteration
    private double maxArea = 0;
    private boolean first = false;

    // Defining an object to synchronize threads
    private final Object sync = new Object();

    // Constructor to initialize the borders of the region of interest
    public ContourPipeline(double borderLeftX, double borderRightX, double borderTopY, double borderBottomY) {
        this.borderLeftX = borderLeftX;
        this.borderRightX = borderRightX;
        this.borderTopY = borderTopY;
        this.borderBottomY = borderBottomY;
    }

    // Methods to configure the lower and upper bounds of the YCrCb color space
    public void configureScalarLower(double y, double cr, double cb) {
        scalarLowerYCrCb = new Scalar(y, cr, cb);
    }

    public void configureScalarUpper(double y, double cr, double cb) {
        scalarUpperYCrCb = new Scalar(y, cr, cb);
    }

    public void configureScalarLower(int y, int cr, int cb) {
        scalarLowerYCrCb = new Scalar(y, cr, cb);
    }

    public void configureScalarUpper(int y, int cr, int cb) {
        scalarUpperYCrCb = new Scalar(y, cr, cb);
    }

    // Method to configure the borders of the region of interest
    public void configureBorders(double borderLeftX, double borderRightX, double borderTopY, double borderBottomY) {
        this.borderLeftX = borderLeftX;
        this.borderRightX = borderRightX;
        this.borderTopY = borderTopY;
        this.borderBottomY = borderBottomY;
    }

    // Overriding the processFrame method to process each frame and find contours
    @Override
    public Mat processFrame(Mat input) {
        // Setting the camera dimensions
        CAMERA_WIDTH = input.width();
        CAMERA_HEIGHT = input.height();
        try {
            // Converting the input image from RGB to YCrCb color space
            Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2YCrCb);
            // Applying a binary threshold to segment the image based on the YCrCb color space
            Core.inRange(mat, scalarLowerYCrCb, scalarUpperYCrCb, processed);
            // Applying morphological operations to remove noise and fill gaps in the segmented image
            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(5, 5));
            Imgproc.morphologyEx(processed, processed, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(processed, processed, Imgproc.MORPH_CLOSE, kernel);

            // Applying Gaussian blur to reduce noise and make the edges smoother
            Imgproc.GaussianBlur(processed, processed, new Size(5, 15), 0.00);

            // Applying adaptive thresholding to segment the image based on the local mean intensity
            Imgproc.adaptiveThreshold(processed, processed, 255, Imgproc.ADAPTIVE_THRESH_MEAN_C, Imgproc.THRESH_BINARY, 11, 2);

            // Finding contours in the segmented image
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(processed, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

            // Drawing the contours on the input image for visualization
            Imgproc.drawContours(input, contours, -1, new Scalar(255, 0, 0));

            // Synchronized block to find the maximum rectangle in the contours
            synchronized (sync) {
                for (MatOfPoint contour : contours) {
                    Point[] contourArray = contour.toArray();

                    // Checking if the contour has at least 15 points
                    if (contourArray.length >= 15) {
                        MatOfPoint2f areaPoints = new MatOfPoint2f(contourArray);
                        Rect rect = Imgproc.boundingRect(areaPoints);
                        // Checking if the rectangle satisfies the conditions for being the maximum rectangle
                        if (rect.area() > maxArea
                                && rect.x + (rect.width / 2.0) > (borderLeftX * CAMERA_WIDTH)
                                && rect.x + (rect.width / 2.0) < CAMERA_WIDTH - (borderRightX * CAMERA_WIDTH)
                                && rect.y + (rect.height / 2.0) > (borderTopY * CAMERA_HEIGHT)
                                && rect.y + (rect.height / 2.0) < CAMERA_HEIGHT - (borderBottomY * CAMERA_HEIGHT)

                                || loopCounter - pLoopCounter > 6
                                && rect.x + (rect.width / 2.0) > (borderLeftX * CAMERA_WIDTH)
                                && rect.x + (rect.width / 2.0) < CAMERA_WIDTH - (borderRightX * CAMERA_WIDTH)
                                && rect.y + (rect.height / 2.0) > (borderTopY * CAMERA_HEIGHT)
                                && rect.y + (rect.height / 2.0) < CAMERA_HEIGHT - (borderBottomY * CAMERA_HEIGHT)
                        ) {
                            // Updating the maximum rectangle and area
                            maxArea = rect.area();
                            maxRect = rect;
                            pLoopCounter++;
                            loopCounter = pLoopCounter;
                            first = true;
                        } else if (loopCounter - pLoopCounter > 10) {
                            // Resetting the maximum rectangle and area if the conditions are not met for 10 iterations
                            maxArea = new Rect().area();
                            maxRect = new Rect();
                        }
                        // Releasing the MatOfPoint2f to free up memory
                        areaPoints.release();
                    }
                    // Releasing the contour to free up memory
                    contour.release();
                }
                // Resetting the maximum rectangle if no contours are found
                if (contours.isEmpty()) {
                    maxRect = new Rect(600, 1, 1, 1);
                }
            }
            // Drawing the maximum rectangle on the input image if it has a sufficient area
            if (first && maxRect.area() > 500) {
                Imgproc.rectangle(input, maxRect, new Scalar(0, 255, 0), 2);
            }
            // Drawing the region of interest on the input image
            Imgproc.rectangle(input, new Rect(
                    (int) (borderLeftX * CAMERA_WIDTH),
                    (int) (borderTopY * CAMERA_HEIGHT),
                    (int) (CAMERA_WIDTH - (borderRightX * CAMERA_WIDTH) - (borderLeftX * CAMERA_WIDTH)),
                    (int) (CAMERA_HEIGHT - (borderBottomY * CAMERA_HEIGHT) - (borderTopY * CAMERA_HEIGHT))
            ), RED, 2);

            // Adding text to display the area and midpoint of the maximum rectangle on the input image
            Imgproc.putText(input, "Area: " + getRectArea() + " Midpoint: " + getRectMidpointXY().x + " , " + getRectMidpointXY().y, new Point(5, CAMERA_HEIGHT - 5), 0, 0.6, new Scalar(255, 255, 255), 2);

            loopCounter++;
        } catch (Exception e) {
            debug = e;
            error = true;
        }
        // Returning the input image with the contours and maximum rectangle drawn on it
        return input;
    }

    // Methods to get the properties of the maximum rectangle (height, width, x, y, midpoint x, midpoint y, midpoint xy, and area)
    public int getRectHeight() {
        synchronized (sync) {
            return maxRect.height;
        }
    }
    public int getRectWidth() {
        synchronized (sync) {
            return maxRect.width;
        }
    }
    public int getRectX() {
        synchronized (sync) {
            return maxRect.x;
        }
    }
    public int getRectY() {
        synchronized (sync) {
            return maxRect.y;
        }
    }

    public double getRectMidpointX() {
        synchronized (sync) {
            return getRectX() + (getRectWidth() / 2.0);
        }
    }
    public double getRectMidpointY() {
        synchronized (sync) {
            return getRectY() + (getRectHeight() / 2.0);
        }
    }
    public Point getRectMidpointXY() {
        synchronized (sync) {
            return new Point(getRectMidpointX(), getRectMidpointY());
        }
    }
    // Method to get the area of the maximum rectangle
    public double getRectArea() {
        synchronized (sync) {
            return maxRect.area();
        }
    }
}