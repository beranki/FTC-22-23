package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp(name = "Mechanism")
public class Mechanism extends LinearOpMode {
    private DcMotor linearmotor;
    private double lowpole = 13.5;
    private double mediumpole = 23.5;
    private double highpole = 33.5;
    private int x = 0;
    private double ticks = 0;
    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    private Servo armServo;
    private int servoState = 2;

    @Override
    public void runOpMode() throws InterruptedException {
        linearmotor = hardwareMap.get(DcMotor.class, "slideMotor");
        armServo = hardwareMap.get(Servo.class, "armServo");
        linearmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.x) {//Carry position
                linearmotor.setTargetPosition((int) (COUNTS_PER_INCH * 2.5 / 2));
                linearmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linearmotor.setPower(0.2);

                if (linearmotor.getCurrentPosition() > (int) (COUNTS_PER_INCH * 2.5) / 2) {
                    linearmotor.setPower(0);
                    linearmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
            } else if (gamepad1.a) {//Low pole accounting for cone height + inches of ground
                linearmotor.setTargetPosition((int) (COUNTS_PER_INCH) * (17 / 2 + 1));
                linearmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linearmotor.setPower(0.2);
                if (linearmotor.getCurrentPosition() > (int) (COUNTS_PER_INCH * 17 / 2 + 1)) {
                    linearmotor.setPower(0);
                    linearmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
            } else if (gamepad1.b) {//Medium pole 23.5 inch plus other factors
                linearmotor.setTargetPosition((int) (COUNTS_PER_INCH * 27 / 2 + 1));
                linearmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linearmotor.setPower(0.2);

                if (linearmotor.getCurrentPosition() > (int) (COUNTS_PER_INCH * 27 / 2 + 1)) {
                    linearmotor.setPower(0);
                    linearmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
            } else if (gamepad1.dpad_down) {
                //either midway up or at its desination
                linearmotor.setTargetPosition(10);
                linearmotor.setPower(-0.2);
                if (linearmotor.getCurrentPosition() <= 10) {
                    linearmotor.setPower(0);
                    linearmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                }
            }


            if (gamepad1.left_bumper) {
                telemetry.addLine("servoOpen");
                telemetry.update();
                armServo.setPosition(0);
            } else {
                telemetry.addLine("servoClose");
                telemetry.update();
                armServo.setPosition(1.0);
            }


            telemetry.addData("MotorEncoder", linearmotor.getCurrentPosition());
            telemetry.addData("x: ", x);
            telemetry.update();
            sleep(20);
        }

    }
}