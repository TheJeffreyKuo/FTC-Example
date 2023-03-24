package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Framework.BaseOpMode;
import org.firstinspires.ftc.teamcode.Framework.Claws;

@TeleOp(name = "Power Play JellyTele")
public class JellyTele extends BaseOpMode {
    protected enum DriveMode {
        TANK,
        DRIVE,
        MECANUM,
        FIELDCENTRIC,
    }
    protected DriveMode driveMode = DriveMode.FIELDCENTRIC;
    public void runOpMode() throws InterruptedException {
        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        initHardware();
        waitForStart();

        while (opModeIsActive()) {
            // DRIVETRAIN
            if (gamepad1.dpad_left) {
                driveMode = DriveMode.TANK;
                gamepad1.rumbleBlips(1);
            } else if (gamepad1.dpad_up) {
                driveMode = DriveMode.MECANUM;
                gamepad1.rumbleBlips(2);
            } else if (gamepad1.dpad_right) {
                driveMode = DriveMode.DRIVE;
                gamepad1.rumbleBlips(3);
            }
            else if (gamepad1.dpad_down) {
                driveMode = DriveMode.FIELDCENTRIC;
                gamepad1.rumbleBlips(4);
            }
            // PRECISION
            double mult = gamepad1.left_bumper ? 0.35 : gamepad1.right_bumper ? 0.7 : 1.0;
            if (gamepad1.left_bumper){
                gamepad1.rumble( 1, 1, 10);
            }
            if (gamepad1.right_bumper){
                gamepad1.rumble(2.7,2.7,10);
            }
            if (gamepad1.y && gamepad1.back){
                imu.resetYaw();
                gamepad1.rumbleBlips(5);
            }
            telemetry.addData("drive mode", driveMode);
            telemetry.addData("mX", gamepad2.left_stick_x);
            telemetry.addData("mY", gamepad2.left_stick_y);
            telemetry.addData("precision mode", mult);
            telemetry.update();
            switch (driveMode) {
                case TANK: {
                    double l = -gamepad1.left_stick_y,
                        r = -gamepad1.right_stick_y;
                    setMotorSpeeds(mult, new double[] {r, r, l, l});
                    break;
                }
                case DRIVE: {
                    double pivot = gamepad1.left_stick_x, y = -gamepad1.left_stick_y;
                    setMotorSpeeds(mult, new double[] {
                        y-pivot,
                        y-pivot,
                        y+pivot,
                        y+pivot
                    });
                    break;
                }
                case MECANUM: {
                    double pivot = gamepad1.right_stick_x;
                    double mX, mY;
                    mX = gamepad1.left_stick_x;
                    mY = -gamepad1.left_stick_y;
                    setMotorSpeeds(mult, new double[] {
                        mY - mX - pivot,
                        mY + mX - pivot,
                        mY + mX + pivot,
                        mY - mX + pivot});
                    break;
                }
                case FIELDCENTRIC: {
                    double y = -gamepad1.left_stick_y; // Remember, this is reversed!
                    double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
                    double rx = gamepad1.right_stick_x;

                    double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

                    // Rotate the movement direction counter to the bot's rotation
                    double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                    double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
                    setMotorSpeeds(mult, new double[] {
                            rotY - rotX - rx,
                            rotY + rotX - rx,
                            rotY + rotX + rx,
                            rotY - rotX + rx});
                    break;
                }
            }
            if(gamepad2.right_bumper){
                Claws.clawsClose();
            }
            if(gamepad2.left_bumper){
                Claws.clawsOpen();
            }
        }
    }
    // CLAW

    // PRECISION METHOD
    protected void setMotorSpeeds(double mult, double[] powers) {
        for (int i = 0; i < 4; i++) {
            powers[i] = powers[i] * mult;
        }
        double max = Math.max(Math.max(Math.abs(powers[0]), Math.abs(powers[1])),
                Math.max(Math.abs(powers[2]), Math.abs(powers[3])));
        double scale = Math.abs(1 / max);
        // SCALES POWER TO KEEP IN RANGE LIMIT
        if (scale > 1) {
            scale = 1;
        }
        for (int i = 0; i < 4; i++) {
            powers[i] *= scale;
        }
        for (int i = 0; i < 4; i++) {
            motors[i].setPower(powers[i]);
        }
    }
}