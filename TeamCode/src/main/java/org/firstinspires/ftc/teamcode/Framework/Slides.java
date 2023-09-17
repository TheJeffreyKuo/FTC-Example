package org.firstinspires.ftc.teamcode.Framework;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Slides {
    // Defining private variables to hold motor objects and constants for the PID controller
    private DcMotor leftSlideMotor;
    private DcMotor rightSlideMotor;
    private final double Kp = 0;
    private final double Ki = 0;
    private final double Kd = 0;
    private final double f = 0;
    public int targetPosition = 0;
    private final double ticksInDegree = 145.1 / 360.0;
    PIDCoefficients coefficients = new PIDCoefficients(Kp,Ki,Kd);
    BasicPID controller = new BasicPID(coefficients);
    private double power;

    // Constructor initializes the Slides object with the left and right slide motors
    public Slides(DcMotor leftSlideMotor, DcMotor rightSlideMotor) {
        this.leftSlideMotor = leftSlideMotor;
        this.rightSlideMotor = rightSlideMotor;
    }

    // Method to set the target position for the slides
    public void setTargetPosition(int targetPosition) {
        this.targetPosition = targetPosition;
    }

    // Method updates the power to the slide motors using PID control and a feed-forward term
    public void update() {
        // Gets the current position of the motor
        int currentPosition = leftSlideMotor.getCurrentPosition();
        // Calculate the PID control output based on the current and target positions
        double pid = controller.calculate(targetPosition, currentPosition);
        // Calculate the feed-forward term to anticipate gravity's effect on the slide
        double ff = Math.cos(Math.toRadians(targetPosition / ticksInDegree)) * f;
        // Determine the total power as the sum of PID output and feed-forward term
        this.power = pid + ff;
        moveSlides(power);
    }

    // Method to set the power of both slide motors
    public void moveSlides(double power) {
        leftSlideMotor.setPower(power);
        rightSlideMotor.setPower(power);
    }
    // Method to get the position of where you want the system to be
    public int getTargetPosition() {
        return this.targetPosition;
    }
}
