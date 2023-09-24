package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@Autonomous(name="RedAuto")

public class RedAuto extends LinearOpMode {
    // Defining a variable for the webcam
    private OpenCvCamera webcam;

    // Defining constants for the camera dimensions
    private static final int CAMERA_WIDTH  = 1280;
    private static final int CAMERA_HEIGHT = 720;

    // Defining variables to store the YCrCb color space thresholds
    private double CrLowerUpdate = 160;
    private double CbLowerUpdate = 100;
    private double CrUpperUpdate = 255;
    private double CbUpperUpdate = 255;

    // Defining variables to store the runtime for updating the YCrCb thresholds
    private double lowerruntime = 0;
    private double upperruntime = 0;

    // Defining static variables to configure the borders of the region of interest
    public static double borderLeftX    = 0.0;
    public static double borderRightX   = 0.0;
    public static double borderTopY     = 0.0;
    public static double borderBottomY  = 0.0;

    // Defining static variables for the lower and upper bounds of the YCrCb color space
    public static Scalar scalarLowerYCrCb = new Scalar(0, 133, 77);
    public static Scalar scalarUpperYCrCb = new Scalar(255, 173, 127);

    @Override
    public void runOpMode()
    {
        // Getting the ID of the camera monitor view
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        // Initializing the webcam with the specified ID and pipeline
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        ContourPipeline myPipeline;
        webcam.setPipeline(myPipeline = new ContourPipeline(borderLeftX,borderRightX,borderTopY,borderBottomY));

        // Configuring the YCrCb color space thresholds in the pipeline
        myPipeline.configureScalarLower(scalarLowerYCrCb.val[0],scalarLowerYCrCb.val[1],scalarLowerYCrCb.val[2]);
        myPipeline.configureScalarUpper(scalarUpperYCrCb.val[0],scalarUpperYCrCb.val[1],scalarUpperYCrCb.val[2]);

        // Opening the camera device asynchronously and starting the streaming
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                // Handling any errors that occur
            }
        });

        // Initializing the FTC dashboard and setting up the telemetry
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(webcam, 10);
        // Updating the telemetry and waiting for the start signal
        telemetry.update();
        waitForStart();

        while (opModeIsActive())
        {
            // Configuring the borders of the region of interest in the pipeline
            myPipeline.configureBorders(borderLeftX, borderRightX, borderTopY, borderBottomY);
            // Checking for errors and displaying debug information
            if(myPipeline.error){
                telemetry.addData("Exception: ", myPipeline.debug);
            }
            // Calling the testing method to update the YCrCb thresholds
            testing(myPipeline);
            // Displaying the area of the maximum rectangle found in the contours
            telemetry.addData("RectArea: ", myPipeline.getRectArea());
            telemetry.update();
            // Getting the x-coordinate of the midpoint of the maximum rectangle
            double rectMidpointX = myPipeline.getRectMidpointX();
            // Determining the autonomous path based on the area and midpoint of the maximum rectangle
            if(myPipeline.getRectArea() > 2000){
                if(rectMidpointX > 400){
                    AUTONOMOUS_C();
                }
                else if(rectMidpointX > 200){
                    AUTONOMOUS_B();
                }
                else {
                    AUTONOMOUS_A();
                }
            }
        }
    }
    // Method to update the YCrCb thresholds based on the gamepad input for tunning
    public void testing(ContourPipeline myPipeline){
        // Updating the lower YCrCb thresholds
        if(lowerruntime + 0.05 < getRuntime()){
            CrLowerUpdate += -gamepad1.left_stick_y;
            CbLowerUpdate += gamepad1.left_stick_x;
            lowerruntime = getRuntime();
        }
        // Updating the upper YCrCb thresholds
        if(upperruntime + 0.05 < getRuntime()){
            CrUpperUpdate += -gamepad1.right_stick_y;
            CbUpperUpdate += gamepad1.right_stick_x;
            upperruntime = getRuntime();
        }
        // Ensuring the YCrCb thresholds are within the valid range
        CrLowerUpdate = inValues(CrLowerUpdate, 0, 255);
        CrUpperUpdate = inValues(CrUpperUpdate, 0, 255);
        CbLowerUpdate = inValues(CbLowerUpdate, 0, 255);
        CbUpperUpdate = inValues(CbUpperUpdate, 0, 255);
        // Configuring the YCrCb thresholds in the pipeline
        myPipeline.configureScalarLower(0.0, CrLowerUpdate, CbLowerUpdate);
        myPipeline.configureScalarUpper(255.0, CrUpperUpdate, CbUpperUpdate);
        // Displaying the YCrCb thresholds on the telemetry
        telemetry.addData("lowerCr ", (int)CrLowerUpdate);
        telemetry.addData("lowerCb ", (int)CbLowerUpdate);
        telemetry.addData("UpperCr ", (int)CrUpperUpdate);
        telemetry.addData("UpperCb ", (int)CbUpperUpdate);
    }
    // Method to constrain a value within a specified range
    public Double inValues(double value, double min, double max){
        if(value < min){ value = min; }
        if(value > max){ value = max; }
        return value;
    }
    // Methods to define the autonomous paths A, B, and C
    public void AUTONOMOUS_A(){
        telemetry.addLine("Autonomous A");
    }
    public void AUTONOMOUS_B(){
        telemetry.addLine("Autonomous B");
    }
    public void AUTONOMOUS_C(){
        telemetry.addLine("Autonomous C");
    }
}