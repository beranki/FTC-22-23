package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ConeScanner extends OpenCvPipeline {
    int coneColor;

    Mat greenMat = new Mat();
    Mat blueMat = new Mat();
    Mat redMat = new Mat();

    int width = 320;
    int height = 240;

    Rect rectMiddleROI = new Rect(new Point(0,0), new Point(width, height));


    // Process each frame
    @Override
    public Mat processFrame(Mat m) {
        Imgproc.cvtColor(m, greenMat, Imgproc.COLOR_RGB2HSV); //range: 0-360
        Imgproc.cvtColor(m, blueMat, Imgproc.COLOR_RGB2HSV);
        Imgproc.cvtColor(m, redMat, Imgproc.COLOR_RGB2HSV);


        Scalar lowGreenHSV = new Scalar(36, 50, 70); //36 for green
        Scalar highGreenHSV = new Scalar(89, 255, 255); //89 for green

        Scalar lowBlueHSV = new Scalar(95, 100, 100);
        Scalar highBlueHSV = new Scalar(128, 255, 255);

        Scalar lowRedHSV = new Scalar(0, 100, 100);
        Scalar highRedHSV = new Scalar(14, 255, 255);

        Core.inRange(greenMat, lowGreenHSV, highGreenHSV, greenMat);
        Core.inRange(blueMat, lowBlueHSV, highBlueHSV, blueMat);
        Core.inRange(redMat, lowRedHSV, highRedHSV, redMat);

        Mat greenmat = greenMat.submat(rectMiddleROI);
        double greenWhitePercent = (Core.sumElems(greenmat).val[0]/rectMiddleROI.area())/255;

        Mat bluemat = blueMat.submat(rectMiddleROI);
        double blueWhitePercent = (Core.sumElems(bluemat).val[0]/rectMiddleROI.area())/255;

        Mat redmat = redMat.submat(rectMiddleROI);
        double redWhitePercent = (Core.sumElems(redmat).val[0]/rectMiddleROI.area())/255;

        if(redWhitePercent>greenWhitePercent && redWhitePercent>blueWhitePercent){
            coneColor = 1; // value is red
        }else if(blueWhitePercent>redWhitePercent && blueWhitePercent>greenWhitePercent){
            coneColor = 2; // value is blue
        }else if(greenWhitePercent>redWhitePercent && greenWhitePercent>blueWhitePercent){
            coneColor = 3; // value is green
        }else{
            coneColor = 4; //value cannot be found
        }

        return m;
    }


    public int coneColor() {
        return coneColor;
    }

}