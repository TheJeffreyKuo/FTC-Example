package org.firstinspires.ftc.teamcode.Framework;

import com.qualcomm.robotcore.hardware.Servo;

public class Claw {

    private Servo claw;

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
