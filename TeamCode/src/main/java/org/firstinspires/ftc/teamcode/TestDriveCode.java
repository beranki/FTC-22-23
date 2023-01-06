package org.firstinspires.ftc.teamcode;

import android.transition.Slide;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp(name = "Test Drive - For collecting encoder values")
public class TestDriveCode extends LinearOpMode {
    private DcMotor tlm, trm, blm, brm, slideMotor;

    int slideStatus = 0; //0 is neutral, -1 is down, +1 is up
    private double basisAngle = 0;
    private int servoState = 2;
    private int armState = 5;
    private Servo armServo;
    private double S = 0.93;
    double ticks = 0;
    private double power = 0.0;
    int st = 0;
    private DigitalChannel maxLimitTouch;

    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)/(WHEEL_DIAMETER_INCHES * 3.1415);
    private int x = 0;
    String lastPressed = "";

    @Override
    public void runOpMode() throws InterruptedException {
        //hex motor 20:1 gearbox -> linear slide; touch sensor for max height

        armServo = hardwareMap.get(Servo.class, "armServo");
        slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");

        tlm = hardwareMap.get(DcMotor.class, "frontLeft");
        trm = hardwareMap.get(DcMotor.class, "frontRight");
        blm = hardwareMap.get(DcMotor.class, "backLeft");
        brm = hardwareMap.get(DcMotor.class, "backRight");

        blm.setDirection(DcMotorSimple.Direction.REVERSE);

        tlm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        trm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        armServo.scaleRange(0.0, 0.8);
        armServo.setDirection(Servo.Direction.REVERSE);

        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeIsActive()) {
            if (gamepad1.right_trigger > 0.02 || gamepad1.left_trigger > 0.02) { //left == ccw rotation, right == cw rotation
                if (gamepad1.right_trigger > 0) {
                    power = 0.2;
                } else if (gamepad1.left_trigger > 0) {
                    power = -0.2;
                }

                trm.setPower(-power);   // abs -> (+) => + power to trm (should be -, refer to cmt above)
                brm.setPower(-power);   // abs -> (+) => + power to brm
                tlm.setPower(power);   // abs -> (+) => - power to tlm (keep in mind left is being fed reverse, so +reverse is -)
                blm.setPower(power);  // abs -> (+) => + power to blm (keep in mind left is being fed reverse, so -reverse is +)
                st = 0;
            } else { //only when there's no response on the triggers

                //so calc for sign isn't undefined, and also falls out of +/- 0.02 margin for x adjustment of joystick
                if (Math.abs(gamepad1.right_stick_x) != 0) {
                    power = 0.2*Math.abs(gamepad1.right_stick_x)/gamepad1.right_stick_x;
                    st = -1;
                    trm.setPower(-power);   // abs -> (+) => + power to trm (should be -, refer to cmt above)
                    brm.setPower(power);  // abs -> (+) => - power to brm
                    tlm.setPower(power);   // abs -> (+) => - power to tlm (keep in mind left is being fed reverse, so -reverse is +)
                    blm.setPower(-power);   // abs -> (+) => - power to blm (keep in mind left is being fed reverse, so -reverse is +)
                }

                //so calc for sign isn't undefined, also falls out of +/- 0.02 margin for y adjustment of joystick
                else if (Math.abs(gamepad1.left_stick_y) != 0) {
                    power = 0.2*Math.abs(gamepad1.left_stick_y)/gamepad1.left_stick_y;
                    st = 1;
                    trm.setPower(-power);    // abs -> (+) => + power to trm
                    brm.setPower(-power);     // abs -> (+) => + power to brm
                    tlm.setPower(-power);     // abs -> (+) => + power to tlm (keep in mind left is being fed reverse, so +reverse is -)
                    blm.setPower(-power);     // abs -> (+) => + power to blm (keep in mind left if being fed reverse, so +reverse is -)

                } else { //accounts for dead margin (x == 0)
                    trm.setPower(0);
                    brm.setPower(0);
                    tlm.setPower(0);
                    blm.setPower(0);
                }

            }

            telemetry.addData("Slide motor encoder", slideMotor.getCurrentPosition());
            telemetry.addData("x: ", x);
            telemetry.addLine("cumulative wheel powers: \ntrm: " + trm.getPower() + "\ntlm: " + tlm.getPower() + "\nbrm: " + brm.getPower() + "\nblm: " + blm.getPower());
            telemetry.addLine("\nservo value: " + armServo);
            telemetry.addData("TRM ENCODER VALUE: ", trm.getCurrentPosition());
            telemetry.update();

            sleep(20);

        }

    }

}