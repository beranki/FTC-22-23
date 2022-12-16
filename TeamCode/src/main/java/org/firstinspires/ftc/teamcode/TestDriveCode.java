package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Drive Test w/ Smoothened Accel")
public class TestDriveCode extends LinearOpMode {
    private DcMotor tlm;
    private DcMotor trm;
    private DcMotor brm;
    private DcMotor blm;
    private double S = 0.95;
    private double power = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        tlm = hardwareMap.get(DcMotor.class, "topleft");
        trm = hardwareMap.get(DcMotor.class, "topright");
        blm = hardwareMap.get(DcMotor.class, "bottomleft");
        brm = hardwareMap.get(DcMotor.class, "bottomright");


        blm.setDirection(DcMotor.Direction.REVERSE);
        tlm.setDirection(DcMotor.Direction.REVERSE);
        tlm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        trm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.right_trigger > 0.02 || gamepad1.left_trigger > 0.02) { //left == ccw rotation, right == cw rotation
                int absR = 0;

                if (gamepad1.right_trigger > 0.02) {
                    power = S*power + (1-S) * gamepad1.right_trigger;
                } else if (gamepad1.left_trigger > 0.02) {
                    power = S*power + (1-S) * gamepad1.left_trigger;
                }

                trm.setPower(-power);   // abs -> (+) => + power to trm (should be -, refer to cmt above)
                brm.setPower(-power);   // abs -> (+) => + power to brm
                tlm.setPower(power);   // abs -> (+) => - power to tlm (keep in mind left is being fed reverse, so +reverse is -)
                blm.setPower(power);  // abs -> (+) => + power to blm (keep in mind left is being fed reverse, so -reverse is +)

            } else { //only when there's no response on the triggers

                //so calc for sign isn't undefined, and also falls out of +/- 0.02 margin for x adjustment of joystick
                if (Math.abs(gamepad1.right_stick_x) > 0.02) {
                    power = S*power + (1-S) * gamepad1.right_stick_x;

                    trm.setPower(-power);   // abs -> (+) => + power to trm (should be -, refer to cmt above)
                    brm.setPower(power);  // abs -> (+) => - power to brm
                    tlm.setPower(power);   // abs -> (+) => - power to tlm (keep in mind left is being fed reverse, so -reverse is +)
                    blm.setPower(-power);   // abs -> (+) => - power to blm (keep in mind left is being fed reverse, so -reverse is +)
                }

                //so calc for sign isn't undefined, also falls out of +/- 0.02 margin for y adjustment of joystick
                else if (Math.abs(gamepad1.left_stick_y) > 0.02) {
                    power = S*power + (1-S) * gamepad1.left_stick_y;

                    telemetry.addData("powY: ", power);
                    trm.setPower(-power);    // abs -> (+) => + power to trm
                    brm.setPower(-power);     // abs -> (+) => + power to brm
                    tlm.setPower(-power);     // abs -> (+) => + power to tlm (keep in mind left is being fed reverse, so +reverse is -)
                    blm.setPower(-power);     // abs -> (+) => + power to blm (keep in mind left if being fed reverse, so +reverse is -)

                } else { //accounts for dead margin (-0.02 <= x <= 0.02)
                    tlm.setPower(0);
                    trm.setPower(0);
                    blm.setPower(0);
                    brm.setPower(0);
                }
            }

        }


    }
}