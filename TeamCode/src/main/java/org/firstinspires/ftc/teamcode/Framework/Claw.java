package org.firstinspires.ftc.teamcode.Framework;

import com.qualcomm.robotcore.hardware.Servo;

public class Claw {

    // Variable Definition
    protected Servo claw;

    // Dependency Injection - The Claw class is being passed a Servo object as a dependency through its constructor
    // Constructor - Variable you created above is initialized through the constructor
    public Claw(Servo servo) {
        this.claw = servo;
    }

    public void clawsOpen() {
        claw.setPosition(1);
    }

    public void clawsClose() {
        claw.setPosition(0);
    }

    public double getCurrentPosition() {
        return claw.getPosition();
    }
}
