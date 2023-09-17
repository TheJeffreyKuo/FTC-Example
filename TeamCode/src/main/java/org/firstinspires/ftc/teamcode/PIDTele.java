package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Framework.BaseOpMode;
@TeleOp(name = "PIDTele")
public class PIDTele extends BaseOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        waitForStart();
        // Main loop for the OpMode
        while(opModeIsActive()){
            // Manuel Joystick control
            slides.setTargetPosition(slides.getTargetPosition() + (int) (gamepad2.left_stick_y * 0.5));
            // Preset positions
            if(gamepad2.a){
                slides.setTargetPosition(0);
            }
            if(gamepad2.b){
                slides.setTargetPosition(100);
            }
            // Updating the slide power using the PID controller inside the loop
            slides.update();
        }
    }
}