package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Autonomous 1.0 -> A2 & F5 (reliable - USE)")
public class
AutonomousBlue1 extends LinearOpMode {
    private DcMotor tlm, trm, blm, brm, slideMotor;
    //private Gyro gyro;
    OpenCvCamera cam;
    private Servo armServo;

    double power = 0.4;
    private int parkingSpot = 0;

    static final double COUNTS_PER_MOTOR_REV = 288;
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    @Override
    public void runOpMode() throws InterruptedException {

        //hex motor 20:1 gearbox -> linear slide; touch sensor for max height

        double colAvg = 0;
        armServo = hardwareMap.get(Servo.class, "armServo");
        slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");

        //gyro = new Gyro(hardwareMap, AngleUnit.DEGREES);

        tlm = hardwareMap.get(DcMotor.class, "frontLeft");
        trm = hardwareMap.get(DcMotor.class, "frontRight");
        blm = hardwareMap.get(DcMotor.class, "backLeft");
        brm = hardwareMap.get(DcMotor.class, "backRight");

        blm.setDirection(DcMotorSimple.Direction.REVERSE);

        tlm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        trm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        trm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        tlm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        blm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armServo.scaleRange(0.0, 0.8);
        armServo.setDirection(Servo.Direction.REVERSE);

        int cameraMonitorViewId;
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        cam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        ConeScanner scanner = new ConeScanner();
        cam.setPipeline(scanner);

        cam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                cam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        waitForStart();

        while (opModeIsActive()) {
            updateServo(true); //close the servo to grip el cone
            sleep(1000);

            slideMotor.setTargetPosition((int) (COUNTS_PER_INCH * 28.5)*720/288);
            slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideMotor.setPower(0.4);
            sleep(1000);
            moveForward(power, (int)(trm.getCurrentPosition() + COUNTS_PER_INCH*15.5));

            if (scanner.coneColor() == 1) {
                parkingSpot = scanner.coneColor();
                telemetry.addLine("red");
            } else if (scanner.coneColor() == 2) {
                parkingSpot = scanner.coneColor();
                telemetry.addLine("blue");
            } else if (scanner.coneColor() == 3) {
                parkingSpot = scanner.coneColor();
                telemetry.addLine("green");
            } else {
                parkingSpot = 2;
                telemetry.addLine("error");
            }

            telemetry.addLine("Parking spot: " + parkingSpot);
            telemetry.update();

            sleep(2000);

            moveBackward(power, (int)(trm.getCurrentPosition() - COUNTS_PER_INCH*14.5));
            sleep(1000);

            //positioning to middle junc
            telemetry.addLine("GOING TO JUNCTION NOW");
            telemetry.update();

            moveLeft(power*1.5, (int)(COUNTS_PER_INCH*24 + trm.getCurrentPosition()));
            moveForward(power, (int)(trm.getCurrentPosition() + COUNTS_PER_INCH*22.5));
            telemetry.addLine(trm.getCurrentPosition() + "");
            telemetry.update();
            sleep(1000);
            moveRight(power*1.5, (int)(trm.getCurrentPosition() - COUNTS_PER_INCH*12.5));
            moveForward(power, (int)(trm.getCurrentPosition() + COUNTS_PER_INCH*3.5));
            sleep(1000);


            //opens servo, leaves it open for a second, then closes it
            updateServo(false);
            sleep(2000);
            updateServo(true);
            sleep(1000);
            moveBackward(power, (int)(trm.getCurrentPosition()-4.5*COUNTS_PER_INCH)); //subject to change

            slideMotor.setPower(-0.5);
            slideMotor.setTargetPosition(0);
            while (slideMotor.getCurrentPosition() >= 0) {
                sleep(20);
                continue;

            }

            moveLeft(power*1.5, (int)(trm.getCurrentPosition() + 13*COUNTS_PER_INCH));
            moveForward(power, (int)(trm.getCurrentPosition() + 23.5*COUNTS_PER_INCH));

            //`parkingSpot = 2;
            if (parkingSpot == 1) {
                moveBackward(power, (int)(trm.getCurrentPosition() - COUNTS_PER_INCH*2));
            }
            if (parkingSpot == 2) {
                moveRight(power*1.5, (int)(trm.getCurrentPosition() - 25*COUNTS_PER_INCH));
            } else if (parkingSpot == 3) {
                moveRight(power*1.5, (int)(trm.getCurrentPosition() - 49*COUNTS_PER_INCH));
            }

            sleep(20);
            cam.stopStreaming();


            break;
        }
    }


    public void updateServo(boolean state) {
        if (state) {
            telemetry.addLine("servoClose");
            telemetry.update();
            armServo.setPosition(0.8);
        } else {
            telemetry.addLine("servoOpen");
            telemetry.update();
            armServo.setPosition(0.1);
        }
    }

    public void resetMotors() {
        trm.setPower(0);
        tlm.setPower(0);
        blm.setPower(0);
        brm.setPower(0);

    }

    public void moveForward(double power, int dist) {
        while (trm.getCurrentPosition() <= dist) {
            telemetry.addData("TRM CURR POS (forward) ", trm.getCurrentPosition());
            telemetry.update();
            trm.setPower(power);    // abs -> (+) => + power to trm
            brm.setPower(power);     // abs -> (+) => + power to brm
            tlm.setPower(power);     // abs -> (+) => + power to tlm (keep in mind left is being fed reverse, so +reverse is -)
            blm.setPower(power);
            sleep(200);
        }

        telemetry.addLine("to be reset (forward)");
        telemetry.update();


        resetMotors();
        sleep(20);
    }

    public void moveBackward(double power, int dist) {
        while (trm.getCurrentPosition() >= dist) {
            telemetry.addData("TRM CURR POS (backward) ", trm.getCurrentPosition());
            telemetry.update();
            trm.setPower(-power);    // abs -> (+) => + power to trm
            brm.setPower(-power);     // abs -> (+) => + power to brm
            tlm.setPower(-power);     // abs -> (+) => + power to tlm (keep in mind left is being fed reverse, so +reverse is -)
            blm.setPower(-power);
            sleep(200);
        }

        telemetry.addLine("to be reset. (backward)");
        telemetry.update();

        resetMotors();
        sleep(20);
    }

    public void moveLeft(double power, int dist) {
        while (trm.getCurrentPosition() <= dist) { //ref the - motor
            telemetry.addData("BRM CURR POS (left) ", brm.getCurrentPosition());
            trm.setPower(power);
            brm.setPower(-power);
            blm.setPower(power);
            tlm.setPower(-power);
            sleep(200);
        }

        telemetry.addLine("to be reset. (left)");
        telemetry.update();

        resetMotors();
        sleep(20);
    }

    public void moveRight(double power, int dist) {
        while (trm.getCurrentPosition() >= dist) { //ref the + motor
            telemetry.addData("BRM CURR POS (right) ", brm.getCurrentPosition());
            trm.setPower(-power);
            brm.setPower(power);
            tlm.setPower(power);
            blm.setPower(-power);
            sleep(200);
        }

        telemetry.addLine("to be reset. (right)");
        telemetry.update();

        resetMotors();
        sleep(20);
    }

}