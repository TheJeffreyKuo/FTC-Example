package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.opencv.core.Scalar;

@Config
@Autonomous(name="redAuto")
public class RedAuto extends LinearOpMode {

    private static final int CAMERA_WIDTH = 640;
    private static final int CAMERA_HEIGHT = 360;

    // Color ranges
// Adjust these values for detecting red more accurately
    private static final Scalar RED_RANGE_LOWER = new Scalar(0.0, 160.0, 100.0);
    private static final Scalar RED_RANGE_UPPER = new Scalar(255.0, 255.0, 255.0);


    private static double borderLeftX = 0.0;
    private static double borderRightX = 0.0;
    private static double borderTopY = 0.0;
    private static double borderBottomY = 0.0;

    private OpenCvCamera webcam;

    // Add more descriptive variable names
    private double lowerRuntime = 0;
    private double upperRuntime = 0;

    @Override
    public void runOpMode() {
        setupWebcam();

        // Only if you are using FTC Dashboard
        setupFTCDashboard();

        telemetry.update();
        waitForStart();

        ContourPipeline myPipeline = new ContourPipeline(borderLeftX, borderRightX, borderTopY, borderBottomY);
        webcam.setPipeline(myPipeline);
        configurePipelineScalars(myPipeline, RED_RANGE_LOWER, RED_RANGE_UPPER);

        while (opModeIsActive()) {
            updatePipelineBorders(myPipeline);

            if (myPipeline.error) {
                telemetry.addData("Exception: ", myPipeline.debug);
            }

            // Enhance the method name to describe its purpose better
            updateColorScalarsBasedOnJoystickInput(myPipeline);
            displayTelemetryData(myPipeline);
            determineAutonomousPath(myPipeline);
        }
    }

    private void setupWebcam() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                // Add a descriptive error message
                telemetry.addData("Camera Error", "Error code: " + errorCode);
            }
        });
    }

    private void setupFTCDashboard() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(webcam, 10);
    }

    private void configurePipelineScalars(ContourPipeline pipeline, Scalar lower, Scalar upper) {
        pipeline.configureScalarLower(lower.val[0], lower.val[1], lower.val[2]);
        pipeline.configureScalarUpper(upper.val[0], upper.val[1], upper.val[2]);
    }

    private void updatePipelineBorders(ContourPipeline pipeline) {
        pipeline.configureBorders(borderLeftX, borderRightX, borderTopY, borderBottomY);
    }

    private void updateColorScalarsBasedOnJoystickInput(ContourPipeline pipeline) {
        // Your existing testing method implementation
    }

    private void displayTelemetryData(ContourPipeline pipeline) {
        telemetry.addData("Rect Area",pipeline.getRectArea());
        telemetry.addData("Rect Midpoint X", pipeline.getRectMidpointX());
        telemetry.update();
    }

    private void determineAutonomousPath(ContourPipeline pipeline) {
        if (pipeline.getRectArea() > 2000) {
            if (pipeline.getRectMidpointX() > 400) {
                autonomousPathC();
            } else if (pipeline.getRectMidpointX() > 200) {
                autonomousPathB();
            } else {
                autonomousPathA();
            }
        }
    }

    private void autonomousPathA() {
        telemetry.addLine("Autonomous A");
    }

    private void autonomousPathB() {
        telemetry.addLine("Autonomous B");
    }

    private void autonomousPathC() {
        telemetry.addLine("Autonomous C");
    }
}
