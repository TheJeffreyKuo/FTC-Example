package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Framework.BaseOpMode;


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

        initHardware();
        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
            RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        waitForStart();

            // DRIVETRAIN
            if (gamepad1.dpad_left) {
                driveMode = DriveMode.TANK;
            } else if (gamepad1.dpad_up) {
                driveMode = DriveMode.MECANUM;
            } else if (gamepad1.dpad_right) {
                driveMode = DriveMode.DRIVE;
            }
            else if (gamepad1.dpad_down) {
                driveMode = DriveMode.FIELDCENTRIC;
            }
            // precision
            double mult = gamepad1.left_bumper ? 0.35 : gamepad1.right_bumper ? 0.7 : 1.0;

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
                    // right = +, left = -
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
                    double y = -gamepad1.left_stick_y; // Remember, this is reversed
                    double x = gamepad1.left_stick_x;
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

    // precision method
    protected void setMotorSpeeds(double mult, double[] powers) {
        for (int i = 0; i < 4; i++) {
            powers[i] = powers[i] * mult;
        }

        double max = Math.max(Math.max(Math.abs(powers[0]), Math.abs(powers[1])),
                Math.max(Math.abs(powers[2]), Math.abs(powers[3])));
        double scale = Math.abs(1 / max);
        // don't increase power, only decrease
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