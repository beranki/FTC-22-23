package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous (name = "Cone Tester")
public class ConeTester extends LinearOpMode {
    OpenCvCamera cam;

    @Override
    public void runOpMode() throws InterruptedException {

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
            telemetry.addData("Cone color", scanner.coneColor);
            if (scanner.coneColor() == 1) {
                telemetry.addLine("red");
            } else if (scanner.coneColor() == 2) {
                telemetry.addLine("blue");
            } else if (scanner.coneColor() == 3) {
                telemetry.addLine("yellow");
            } else {
                telemetry.addLine("error");
            }
            telemetry.update();
            sleep(500);
        }

        cam.stopStreaming();
    }
}