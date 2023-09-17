package org.firstinspires.ftc.teamcode.Framework;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public abstract class BaseOpMode extends LinearOpMode {
    // Variable Definition - Define arrays to hold motor objects and instances of Claw and Slides classes
    protected DcMotor[] motors;
    protected Claw claws;
    protected Slides slides;
    // Method to initialize hardware components in OpModes
    protected void initHardware() {
        // Motor initialization by getting them from the hardwareMap in array
        motors = new DcMotor[]{
                hardwareMap.dcMotor.get("motor fr"),
                hardwareMap.dcMotor.get("motor br"),
                hardwareMap.dcMotor.get("motor fl"),
                hardwareMap.dcMotor.get("motor bl")
        };

        // Based on the physical orientation of the motor on the robot, it sets the desired direction of the motor
        motors[0].setDirection(DcMotor.Direction.REVERSE);
        motors[1].setDirection(DcMotor.Direction.REVERSE);
        motors[2].setDirection(DcMotor.Direction.FORWARD);
        motors[3].setDirection(DcMotor.Direction.FORWARD);

        // Initialize the servo hardware
        Servo claw = hardwareMap.get(Servo.class, "servo");
        // Create a Claw object and assign in to the claws variable (From the constructor inside the Claw class)
        claws = new Claw(claw);

        // Getting references to the slide motor hardware and initializing Slides object with them
        DcMotor leftSlideMotor = hardwareMap.get(DcMotor.class, "leftSlideMotor");
        DcMotor rightSlideMotor = hardwareMap.get(DcMotor.class, "rightSlideMotor");
        slides = new Slides(leftSlideMotor, rightSlideMotor);
    }
}
