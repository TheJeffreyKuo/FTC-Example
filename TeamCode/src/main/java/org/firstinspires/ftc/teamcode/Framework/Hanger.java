package org.firstinspires.ftc.teamcode.Framework;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
public class Hanger {

    private DcMotor hanger;
    public Hanger(DcMotor hanger) {
        this.hanger = hanger;
    }

}
