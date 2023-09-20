package org.firstinspires.ftc.teamcode;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp(name = "PIDF Test")
public class PIDF_Test extends LinearOpMode {
    // Public static variables for tuning PIDF coefficients through the dashboard
    public static double p = 0, i = 0, d = 0;
    public static double f = 0;
    public static int targetPosition = 0;

    // Conversion factor for ticks to degrees
    private final double ticksInDegree = 145.1 / 360.0;

    // Motor declarations
    private DcMotorEx slideRight, slideLeft;

    // PID coefficients object initialization
    PIDCoefficients coefficients = new PIDCoefficients(p, i, d);

    @Override
    public void runOpMode() throws InterruptedException {
        // Setting up telemetry with the FTC dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Mapping and initializing motors
        slideRight = hardwareMap.get(DcMotorEx.class, "rightSlide");
        slideLeft = hardwareMap.get(DcMotorEx.class, "leftSlide");
        slideRight.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        while (opModeIsActive()) {
            // Re-initializing the PID controller with the latest coefficients
            BasicPID controller = new BasicPID(coefficients);

            // Getting the current position of the right slide
            int currentPosition = slideRight.getCurrentPosition();

            // Calculating PID output using the target and current positions
            double pid = controller.calculate(targetPosition, currentPosition);

            // Calculating feed-forward term using the cosine of the target angle
            double ff = Math.cos(Math.toRadians(targetPosition / ticksInDegree)) * f;

            // Combining PID and feed-forward terms to get the total power
            double power = pid + ff;

            // Setting the motor powers with the calculated power
            slideRight.setPower(power);
            slideLeft.setPower(power);

            // Sending the current state to the telemetry
            telemetry.addData("right_pos", slideRight.getCurrentPosition());
            telemetry.addData("left_pos", slideLeft.getCurrentPosition());
            telemetry.addData("target", targetPosition);
            telemetry.update();
        }
    }
}
