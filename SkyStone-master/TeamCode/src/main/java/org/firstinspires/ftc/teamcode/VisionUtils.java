package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.os.Environment;

import com.vuforia.Image;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.io.File;
import java.io.FileOutputStream;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;


/**
 * Created on 9/13/2017.
 */

public class VisionUtils {
    /**
     * Saves an inputted bitmap to a directory
     *
     * @param bitmap  Input bitmap
     * @param name    Name of the saved file
     * @param fileDir The name of the file directory for the image to be saved to
     */
    public static void saveImageToFile(Bitmap bitmap, String name, String fileDir) {

        String root = Environment.getExternalStorageDirectory().getAbsolutePath();
        File myDir = new File(root + fileDir);
        myDir.mkdirs();

        String fname = name + ".jpg";
        File file = new File(myDir, fname);
        if (file.exists()) file.delete();
        try {
            FileOutputStream out = new FileOutputStream(file);
            bitmap.compress(Bitmap.CompressFormat.JPEG, 100, out);
            out.flush();
            out.close();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    /**
     * Converts a bitmap and a type to a openCv mat
     *
     * @param bit    the input bitmap
     * @param cvType the mat type
     * @return output mat
     */
    public static Mat bitmapToMat(Bitmap bit, int cvType) {
        Mat newMat = new Mat(bit.getHeight(), bit.getWidth(), cvType);
        Utils.bitmapToMat(bit, newMat);
        return newMat;
    }

    /**
     * Converts a openCv mat to a bitmap
     *
     * @param mat input mat
     * @return output bitmap
     */
    public static Bitmap matToBitmap(Mat mat) {
        Bitmap newBit = Bitmap.createBitmap(mat.cols(), mat.rows(), Bitmap.Config.ARGB_8888);
        Utils.matToBitmap(mat, newBit);
        return newBit;
    }

    public static Image getImageFromFrame(VuforiaLocalizer.CloseableFrame frame, int format) {

        long numImgs = frame.getNumImages();
        for (int i = 0; i < numImgs; i++) {
            if (frame.getImage(i).getFormat() == format) {
                return frame.getImage(i);
            }//if
        }//for

        return null;
    }

    public static void refreshKernel(Mat kernel, int height, int width, int TARGETHEIGHT, int TARGETWIDTH) {

        if (kernel == null || width != TARGETWIDTH || height != TARGETHEIGHT) {
            if (kernel != null) {
                kernel.release();
            }
            kernel = Mat.ones(TARGETHEIGHT, TARGETWIDTH, CvType.CV_8U);
        }

    }

    /**
     * Method to find the centroid of a binary Mask
     *
     * @param mask the input binary mask
     * @return a openCv point with the x and y values of the centroid
     */
    public static Point maskCentroid(Mat mask) {
        Moments mmnts = Imgproc.moments(mask, true);
        return new Point(mmnts.get_m10() / mmnts.get_m00(), mmnts.get_m01() / mmnts.get_m00());
    }

    public static double hsvToTotalAreaInMask(Mat input, Scalar low, Scalar high, String name){
        Mat mask = new Mat();
        Core.inRange(input, low, high, mask);
        //PSVisionUtils.saveImageToFile(PSVisionUtils.matToBitmap(mask),"R4-mask"+ name, "/saved_images");

        //Contours
        List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
        Imgproc.findContours(mask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);


        double totalArea = 0;
        Iterator<MatOfPoint> each = contours.iterator();
        while (each.hasNext()) {
            MatOfPoint wrapper = each.next();
            double area = Imgproc.contourArea(wrapper);
            totalArea += area;
        }
        return totalArea;
    }
    public static int maskSizeInMat(Mat frame, Scalar low, Scalar high) {
        Mat crop = frame.clone();
        Imgproc.cvtColor(crop, crop, Imgproc.COLOR_RGB2HSV);
        Mat mask = new Mat();
        Core.inRange(crop, low, high, mask);
        return Core.countNonZero(mask);
    }

}