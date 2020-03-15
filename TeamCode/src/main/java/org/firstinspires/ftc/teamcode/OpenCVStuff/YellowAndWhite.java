package org.firstinspires.ftc.teamcode.OpenCVStuff;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@TeleOp
public class YellowAndWhite extends LinearOpMode {

    OpenCvCamera phoneCam;

    @Override
    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        phoneCam.openCameraDevice();

        phoneCam.setPipeline(new YellowAndWhite.Pipeline());

        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        waitForStart();

        while(opModeIsActive()) {

            telemetry.addData("Frame Count", phoneCam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", phoneCam.getFps()));
            telemetry.addData("Total frame time ms", phoneCam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", phoneCam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", phoneCam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", phoneCam.getCurrentPipelineMaxFps());
            telemetry.update();

        }

    }

    class Pipeline extends OpenCvPipeline {

        private List<MatOfPoint> whiteContours(Mat input) {

            List<Mat> channels = new ArrayList<>();
            Core.split(input, channels);

            Mat inputRed = new Mat();
            Mat inputGreen = new Mat();
            Mat inputBlue = new Mat();

            Mat whiteMask = new Mat();

            Imgproc.threshold(channels.get(0), inputRed, 160, 255, Imgproc.THRESH_BINARY);
            Imgproc.threshold(channels.get(1), inputGreen, 160, 255, Imgproc.THRESH_BINARY);
            Imgproc.threshold(channels.get(2), inputBlue, 160, 255, Imgproc.THRESH_BINARY);

            Core.bitwise_and(inputRed, inputGreen, whiteMask);
            Core.bitwise_and(inputBlue, whiteMask, whiteMask);

            inputRed.release();
            inputGreen.release();
            inputBlue.release();

            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();

            Imgproc.findContours(whiteMask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            return contours;

        }

        private List<MatOfPoint> yellowContours(Mat input) {

            Mat yellowMask = new Mat(input.size(), 0);

            Mat inputRed = new Mat();
            Mat inputGreen = new Mat();
            Mat inputBlue = new Mat();

            Imgproc.GaussianBlur(input,input,new Size(3,3),0);

            List<Mat> channels = new ArrayList<>();

            Core.split(input,channels);

            Imgproc.threshold(channels.get(0),inputRed,140,255,Imgproc.THRESH_BINARY);
            Imgproc.threshold(channels.get(1),inputGreen,140,255,Imgproc.THRESH_BINARY);
            Imgproc.threshold(channels.get(2),inputBlue,40,255,Imgproc.THRESH_BINARY_INV);

            Core.bitwise_and(inputRed,inputGreen,yellowMask);
            Core.bitwise_and(yellowMask,inputBlue,yellowMask);

            List<MatOfPoint> yellowContours = new ArrayList<>();
            Mat hierarchy = new Mat();

            Imgproc.findContours(yellowMask, yellowContours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            return yellowContours;

        }

        @Override
        public Mat processFrame(Mat input) {

            List<MatOfPoint> whiteContours = whiteContours(input);
            List<MatOfPoint> yellowContours = yellowContours(input);

            for(MatOfPoint contour : whiteContours) {

                Point center = new Point();
                float[] radius = new float[1];

                MatOfPoint2f polyApprox = new MatOfPoint2f();
                Imgproc.approxPolyDP(new MatOfPoint2f(contour.toArray()), polyApprox, 3, true);

                Imgproc.minEnclosingCircle(polyApprox, center, radius);

                if (radius[0] > 12) {
                    Imgproc.circle(input, center, (int) radius[0], new Scalar(0, 0, 200), 2);
                }

            }

            Rect[] boundingRects = new Rect[yellowContours.size()];

            for (int i = 0; i < yellowContours.size(); i++) {

                boundingRects[i] = Imgproc.boundingRect(yellowContours.get(i));

                if(boundingRects[i].area() > 200) {
                    Imgproc.rectangle(input, Imgproc.boundingRect(yellowContours.get(i)), new Scalar(200, 0, 0), 2);
                }
            }


            return input;
        }
    }

}
