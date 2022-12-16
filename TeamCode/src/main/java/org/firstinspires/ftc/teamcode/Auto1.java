/*package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous (name = "Autonomous 1.0")
public class Auto1 extends LinearOpMode {
    OpenCvCamera cam;

    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId;
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        cam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        DetectSleeve scanner = new DetectSleeve(telemetry);
        cam.setPipeline(scanner);

        cam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                cam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });

        waitForStart();

        while (opModeIsActive()) {
            detectCone(scanner);
        }

        cam.stopStreaming();
    }

    public void detectCone(DetectSleeve scanner) {
        telemetry.addData("Cone location", scanner.coneLocation());
        if (scanner.coneLocation() == 1) {
            telemetry.addLine("red");
        } else if (scanner.coneLocation() == 2) {
            telemetry.addLine("blue");
        } else if (scanner.coneLocation() == 3) {
            telemetry.addLine("yellow");
        } else {
            telemetry.addLine("error");
        }
        telemetry.update();
        sleep(500);

    }

}*/