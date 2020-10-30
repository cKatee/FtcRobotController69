/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.vision;


import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import static org.opencv.core.Core.inRange;
import static org.opencv.core.Core.minMaxLoc;

public class OpenCVHIGHGOAL extends OpenCvPipeline {
    boolean viewportPaused = false;
    public static boolean forBlue = true;
    public static Point targetPoint = new Point(0,0);
    /*
     * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
     * highly recommended to declare them here as instance variables and re-use them for
     * each invocation of processFrame(), rather than declaring them as new local variables
     * each time through processFrame(). This removes the danger of causing a memory leak
     * by forgetting to call mat.release(), and it also reduces memory pressure by not
     * constantly allocating and freeing large chunks of memory.
     */
    @Override
    public Mat processFrame(Mat input)
    {
        /*
         * IMPORTANT NOTE: the input Mat that is passed in as a parameter to this method
         * will only dereference to the same image for the duration of this particular
         * invocation of this method. That is, if for some reason you'd like to save a copy
         * of this particular frame for later use, you will need to either clone it or copy
         * it to another Mat.
         */
        Scalar lowerBounds;
        Scalar upperBounds;
        if (forBlue) {
            lowerBounds = new Scalar(92,83,10,255);
            upperBounds = new Scalar(255,200,70,255);
        } else {
            lowerBounds = new Scalar(10,60,60,255);
            upperBounds = new Scalar(80,100,255,255);
        }
        Imgproc.GaussianBlur(input, input, new Size(5,5),0);


        inRange(input,lowerBounds,upperBounds,input);

        Imgproc.GaussianBlur(input, input, new Size(5,5),0);

        Core.MinMaxLocResult locationOfGoal = minMaxLoc(input);
        System.out.println("target is at: x" + locationOfGoal.maxLoc.x + ", y" + locationOfGoal.maxLoc.y);
        System.out.println(Core.getVersionString());
        targetPoint = locationOfGoal.maxLoc;
        /*
         * Draw a simple box around the middle 1/2 of the entire frame
         */

        Imgproc.rectangle(
                input,
                new Point(locationOfGoal.maxLoc.x + 40,
                        locationOfGoal.maxLoc.y),
                new Point(
                        locationOfGoal.maxLoc.x - 80,
                        locationOfGoal.maxLoc.y + 40),
                new Scalar(255, 255, 255), 4);


        /**
         * NOTE: to see how to get data from your pipeline to your OpMode as well as how
         * to change which stage of the pipeline is rendered to the viewport when it is
         * tapped, please see {@link PipelineStageSwitchingExample}
         */

        return input;
    }


}

