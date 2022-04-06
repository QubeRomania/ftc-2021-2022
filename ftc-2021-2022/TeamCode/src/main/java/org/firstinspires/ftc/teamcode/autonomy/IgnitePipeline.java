package org.firstinspires.ftc.teamcode.autonomy;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;


public class IgnitePipeline extends OpenCvPipeline {

    private final Telemetry telemetry;
    double result;

    public enum FreightFrenzyPosition
    {
        LEFT,
        CENTER,
        RIGHT
    }

    private volatile FreightFrenzyPosition position = FreightFrenzyPosition.LEFT;

    // Working Mat variables
    Mat blur = new Mat();
    Mat hsv = new Mat();
    Mat channel = new Mat();
    Mat thold = new Mat();
    Mat region1_Cb, region2_Cb, region3_Cb;
    int avg1, avg2, avg3;


    // Drawing variables
    final Scalar BLUE = new Scalar(0, 0, 255);
    final Scalar GREEN = new Scalar(0, 255, 0);


    static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(20,150);
    static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(140,150);
    static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(260,150);
    static final int REGION_WIDTH = 40;
    static final int REGION_HEIGHT = 40;

    Point region1_pointA = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x,
            REGION1_TOPLEFT_ANCHOR_POINT.y);
    Point region1_pointB = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    Point region2_pointA = new Point(
            REGION2_TOPLEFT_ANCHOR_POINT.x,
            REGION2_TOPLEFT_ANCHOR_POINT.y);
    Point region2_pointB = new Point(
            REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    Point region3_pointA = new Point(
            REGION3_TOPLEFT_ANCHOR_POINT.x,
            REGION3_TOPLEFT_ANCHOR_POINT.y);
    Point region3_pointB = new Point(
            REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    public IgnitePipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {

        // Img processing
        Imgproc.medianBlur(input, blur, 5);
        Imgproc.cvtColor(blur, hsv, Imgproc.COLOR_RGB2HSV);
        Core.extractChannel(hsv, channel, 1);
        Imgproc.threshold(channel, thold, 120, 255, Imgproc.THRESH_BINARY);

        region1_Cb = thold.submat(new Rect(region1_pointA, region1_pointB));
        region2_Cb = thold.submat(new Rect(region2_pointA, region2_pointB));
        region3_Cb = thold.submat(new Rect(region3_pointA, region3_pointB));

        avg1 = (int) Core.mean(region1_Cb).val[0];
        avg2 = (int) Core.mean(region2_Cb).val[0];
        avg3 = (int) Core.mean(region3_Cb).val[0];

        Imgproc.rectangle(
                input, // Buffer to draw on
                region1_pointA, // First point which defines the rectangle
                region1_pointB, // Second point which defines the rectangle
                BLUE, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines

        /*
         * Draw a rectangle showing sample region 2 on the screen.
         * Simply a visual aid. Serves no functional purpose.
         */
        Imgproc.rectangle(
                input, // Buffer to draw on
                region2_pointA, // First point which defines the rectangle
                region2_pointB, // Second point which defines the rectangle
                BLUE, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines

        /*
         * Draw a rectangle showing sample region 3 on the screen.
         * Simply a visual aid. Serves no functional purpose.
         */
        Imgproc.rectangle(
                input, // Buffer to draw on
                region3_pointA, // First point which defines the rectangle
                region3_pointB, // Second point which defines the rectangle
                BLUE, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines


        /*
         * Find the max of the 3 averages
         */
        int maxOneTwo = Math.max(avg1, avg2);
        int max = Math.max(maxOneTwo, avg3);

        /*
         * Now that we found the max, we actually need to go and
         * figure out which sample region that value was from
         */

        if (max == avg1) // Was it from region 1?
        {
            result = 0;
            
            position = FreightFrenzyPosition.LEFT;

            /*
             * Draw a solid rectangle on top of the chosen region.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill
        } else if (max == avg2) // Was it from region 2?
        {
            result = 1;

            position = FreightFrenzyPosition.CENTER;

            /*
             * Draw a solid rectangle on top of the chosen region.
             * Simply a visual aid. Serves no functional purpose.
             */

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region2_pointA, // First point which defines the rectangle
                    region2_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill
        } else if (max == avg3) // Was it from region 3?
        {
            result = 2;

            position = FreightFrenzyPosition.RIGHT;

            /*
             * Draw a solid rectangle on top of the chosen region.
             * Simply a visual aid. Serves no functional purpose.
             */

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region3_pointA, // First point which defines the rectangle
                    region3_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill
        }

        /*
         * Render the 'input' buffer to the viewport. But note this is not
         * simply rendering the raw camera feed, because we called functions
         * to add some annotations to this buffer earlier up.
         */
        
        return input;
    }
    
    public FreightFrenzyPosition getAnalysis()
    {
        return position;
    }
}
