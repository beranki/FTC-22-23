package org.firstinspires.ftc.teamcode;

/*import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;


public class DetectNumCones extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();

    public enum Location {
        MIDDLE,
        NOT_FOUND
    }

    private Location l;
    static final Rect MIDDLE_ROI = new Rect(
            new Point(80, 35),
            new Point(180, 105)
    );

    public DetectNumCones(Telemetry t) {
        telemetry = t;
    }

    @Override
    public Mat processFrame(Mat input) {
        Mat gray, edges, result, v_kernel, d_kernel;
        ArrayList<Mat> cnts;
        gray = new Mat();
        edges = new Mat();
        result = new Mat();
        v_kernel = new Mat();
        d_kernel = new Mat();

    /*
    gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 400, 500)
    result = image.copy()

    vertical_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (1,10))
    detect_vertical = cv2.morphologyEx(edges, cv2.MORPH_OPEN, vertical_kernel, iterations=2)
    cnts = cv2.findContours(detect_vertical, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    cnts = cnts[0] if len(cnts) == 2 else cnts[1]

    for c in cnts:
        cv2.drawContours(result, [c], -1, (36,255,12), 2)

    cv2.putText(result, text=f"cones: {len(cnts)}", org=(0,50), fontFace=cv2.FONT_HERSHEY_COMPLEX, fontScale=1, color=(0, 255, 0),thickness=3)

    return result
    */

/*
    }

    public Mat detectNumCones(Mat image) {
        Mat gray = new Mat();
        Imgproc.cvtColor(image, gray, Imgproc.COLOR_BGR2GRAY);
        //Imgproc.Canny(image, );
    }

    public boolean conesPresent(Mat image) {
        Imgproc.cvtColor(image, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lbHSV = new Scalar(110, 13, 7);
        Scalar ubHSV = new Scalar(245, 65, 54);

        Mat mask = new Mat();
        Core.inRange(mat, lbHSV, ubHSV, mask);
        Mat middle = mat.submat(MIDDLE_ROI);
        double midVal = Core.sumElems(middle).val[0] / MIDDLE_ROI.area() / 255;


        telemetry.addData("Raw value", (int) Core.sumElems(middle).val[0]);
        telemetry.addData("Percentage", Math.round(midVal * 100) + "%");
        middle.release();

    }


}
*/
/*
public class DetectSleeve extends OpenCvPipeline {
    Telemetry telemetry;
    Mat yellowMat = new Mat();
    Mat redMat = new Mat();
    Mat blueMat = new Mat();

    int coneLocation;
    int width = 320;
    int height = 240;

    // (0, 0) is top left of the entire camera view
    Rect MIDDLE_ROI = new Rect(new Point(0, 0), new Point(width, height));
    public enum Location {
        MIDDLE,
        NOT_FOUND
    }

    private Location l;

    public DetectSleeve(Telemetry t) {
        telemetry = t;
    }

    // process each frame
    @Override
    public Mat processFrame(Mat m) {
        // m is the RGB matrix that the camera sees
        // converting matrix from RGB --> HSV
        // HSV: hue - color, saturation - intensity, value - brightness
        Imgproc.cvtColor(m, yellowMat, Imgproc.COLOR_RGB2HSV); // range: 0-360
        Imgproc.cvtColor(m, redMat, Imgproc.COLOR_RGB2HSV);
        Imgproc.cvtColor(m, blueMat, Imgproc.COLOR_RGB2HSV);

        // blue
        Scalar lowHSVBlue = new Scalar(90, 100, 100);
        Scalar highHSVBlue = new Scalar(128, 255, 255);

        // red
        Scalar lowHSVRed = new Scalar(0, 100, 100);
        Scalar highHSVRed = new Scalar(14, 255, 255);

        // yellow
        Scalar lowYellowHSV = new Scalar(20, 100, 100);
        Scalar highYellowHSV = new Scalar(30, 255, 255);

        // convert red, yellow, blue matrix
        Core.inRange(yellowMat, lowYellowHSV, highYellowHSV, yellowMat);
        Core.inRange(redMat, lowHSVRed, highHSVRed, redMat);
        Core.inRange(blueMat, lowHSVBlue, highHSVBlue, blueMat);

        Mat red_mat = redMat.submat(MIDDLE_ROI);
        Mat yellow_mat = yellowMat.submat(MIDDLE_ROI);
        Mat blue_mat = blueMat.submat(MIDDLE_ROI);

        double blue_white_percent = Core.sumElems(blue_mat).val[0] / MIDDLE_ROI.area() / 255;
        double red_white_percent = Core.sumElems(red_mat).val[0] / MIDDLE_ROI.area() / 255;
        double yellow_white_percent = Core.sumElems(yellow_mat).val[0] / MIDDLE_ROI.area() / 255;

        // to prevent memory leaks
        red_mat.release();
        blue_mat.release();
        yellow_mat.release();

        // whichever color is the most prominent is the cone's color value/location thing
        if (red_white_percent > blue_white_percent && red_white_percent > yellow_white_percent) {
            coneLocation = 1;
        } else if (blue_white_percent > yellow_white_percent && blue_white_percent > red_white_percent) {
            coneLocation = 2;
        } else if (yellow_white_percent > blue_white_percent && yellow_white_percent > red_white_percent) {
            coneLocation = 3;
        } else {
            coneLocation = 0;
        }

        return redMat;
    }

    public int coneLocation() {
        return coneLocation;
    }

}
*/