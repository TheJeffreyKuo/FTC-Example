package org.firstinspires.ftc.teamcode.Framework;

import com.qualcomm.robotcore.hardware.Servo;

public class Claws {
    static Servo servo;
    public Claws(Servo servo){
        Claws.servo = servo;
    }

    public static void clawsOpen(){
        servo.setPosition(1);
    }
    public static void clawsClose(){
        servo.setPosition(0);
    }

    public double getCurrentPosition(){
        return servo.getPosition();
    }
}
