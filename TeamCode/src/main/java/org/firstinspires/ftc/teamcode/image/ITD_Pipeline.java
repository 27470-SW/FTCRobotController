package org.firstinspires.ftc.teamcode.image;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

/**
* RingPipeline class.
*
* <p>An OpenCV pipeline generated by GRIP.
*
* @author GRIP
*/
@SuppressWarnings("unused")
public class ITD_Pipeline
{
	private static final String TAG = "SJH_PPL";
	final int channelDepth = 3;

	private Mat resizeImageOutput;
	private Mat roiMat;
	private Mat blurOutput;
	private final int PIPELINE_PASSES = 1;
	private final Mat[] hsvThresholdOutput = new Mat[PIPELINE_PASSES];
	private final Mat[] cvErodeOutput = new Mat[PIPELINE_PASSES];
	private final Mat[] cvDilateOutput = new Mat[PIPELINE_PASSES];

	private final ArrayList<MatOfPoint>[] findContoursOutput= new ArrayList[PIPELINE_PASSES];
	private final ArrayList<MatOfPoint>[] convexHullsOutput = new ArrayList[PIPELINE_PASSES];
	private final ArrayList<MatOfPoint>[] filterContoursOutput = new ArrayList[PIPELINE_PASSES];

	private final Mat cvDilateKernel = new Mat();
	private final Point cvDilateAnchor = new Point(-1, -1);
	private final Scalar cvDilateBorderValue = new Scalar(-1);

	public int resizeImageWidth;
	public int resizeImageHeight;
	private Rect roiRect = new Rect(0, 0, RobotConstants.IP_CAM_WID, RobotConstants.IP_CAM_WID);
	private boolean crop = false;

	public enum PipeLineStage
	{
		RAW,
		SIZE,
		HSV,
/*      BLUR, */
        ERODE,
        DILATE,
        CONTOURS,
        CONVEX,
		FILTER
	}

	@SuppressWarnings("FieldCanBeLocal")
	private String name = "MEC1";
	private boolean logging = true;

	public void setName(String name) {this.name = name;}
	public void setLogging(boolean enableLogging)
	{
		logging = enableLogging;
	}

	public void setCrop(boolean crop) {this.crop = crop;}

	/* ROI = region of interest */
	public void init(Mat mat)
	{
		double TOP = 0.0;
		double BOT = 1.0;
		double LFT = 0.0;
		double RGT = 1.0;
        /* 3 Matrixs for each passes of the pipeline */
		for (int i = 0; i < PIPELINE_PASSES; i++)
		{
			findContoursOutput[i] = new ArrayList<>();
			convexHullsOutput[i] = new ArrayList<>();
			filterContoursOutput[i] = new ArrayList<>();

			hsvThresholdOutput[i] = new Mat();
			cvErodeOutput[i] = new Mat();
			cvDilateOutput[i] = new Mat();

		}

		if(crop)
		{
			TOP = RobotConstants.IP_IMG_TOP;
			BOT = RobotConstants.IP_IMG_BOT;
			LFT = RobotConstants.IP_IMG_LFT;
			RGT = RobotConstants.IP_IMG_RGT;
		}

		if(logging) 
        {
            RobotLog.dd(TAG, " roiMat image top, bot, lft, rgt: %.2f %.2f %.2f %.2f",
                       TOP, BOT, LFT,RGT);
        }
		int roiX = (int)((LFT * (double)mat.width()));
		int roiY = (int)(TOP * (double)mat.height());
		int roiW = (int)((RGT-LFT) * (double)mat.width());
		int roiH = (int)((BOT-TOP) * (double)mat.height());

		if(logging) RobotLog.dd(TAG, " roiMat image x,y: %d %d w,h: %dx%d", roiX, roiY, roiW,roiH);
		roiRect = new Rect(roiX, roiY, roiW, roiH);
		roiMat = new Mat(mat, roiRect);
		resizeImageWidth  = (int)(RobotConstants.IP_CAM_HGT * (RGT-LFT));
		resizeImageHeight = (int)(resizeImageWidth*((double)roiMat.height()/roiMat.width()));
		resizeImageOutput = new Mat(resizeImageHeight, resizeImageWidth, mat.type());
	}

	void sizeSource(Mat source0)
	{
		if(logging) RobotLog.dd(TAG, "Resizing/Cropping image WXH=%dx%d to %dx%d",
								source0.cols(), source0.rows(),
								resizeImageWidth, resizeImageHeight);

		resizeImage(roiMat, resizeImageWidth, resizeImageHeight,
								Imgproc.INTER_LINEAR, resizeImageOutput);
	}

	/**
	 * @ applyImageFilters: This is the primary method that runs the entire pipeline and updates the outputs.
	 * Processes sized Image and configures the HSV Threshold stage with 1 of 3 pre-program HSV ranges.
	 * Power Play Team Signal Sleeve has 1 of 3 colors: Orange, Purple, Green
	 * HSV Threshold->CV Erode->CV Dialte->Find Contours->Convex Hulls->Filter Contours
	 * */
	void applyImageFilters(Mat imageSource, ITD_Detector.Color hsvColor)
	{
		RobotLog.ii(TAG, "FfPipeline.applyImageFilters");
		Mat blurInput = imageSource;
		BlurType blurType = BlurType.get("Gaussian Blur");
		double blurRadius = 4.0;
		if(blurOutput == null)
			blurOutput = new Mat(blurInput.rows(), blurInput.cols(), blurInput.type());

		blur(blurInput, blurType, blurRadius, blurOutput);

		// Step HSV_Threshold0:
		int pipelinePass = 0;

		// Mat hsvThresholdInput = imageSource;

		double[] hsvThresholdHue = new double[2];
		double[] hsvThresholdSaturation = new double[2];
		double[] hsvThresholdValue = new double[2];

		switch(hsvColor)
		{
			case RED:
				RobotLog.ii(TAG, "FfPipeline.RED");
				hsvThresholdHue[0] = RobotConstants.IP_HUE_MIN_RED;
				hsvThresholdHue[1] = RobotConstants.IP_HUE_MAX_RED;
				hsvThresholdSaturation[0] = RobotConstants.IP_SAT_MIN_RED;
				hsvThresholdSaturation[1] = RobotConstants.IP_SAT_MAX_RED;
				hsvThresholdValue[0] = RobotConstants.IP_VAL_MIN_RED;
				hsvThresholdValue[1] = RobotConstants.IP_VAL_MAX_RED;

				break;

			case BLUE:
				RobotLog.ii(TAG, "FfPipeline.BLUE");
				hsvThresholdHue[0] = RobotConstants.IP_HUE_MIN_BLUE;
				hsvThresholdHue[1] = RobotConstants.IP_HUE_MAX_BLUE;
				hsvThresholdSaturation[0] = RobotConstants.IP_SAT_MIN_BLUE;
				hsvThresholdSaturation[1] = RobotConstants.IP_SAT_MAX_BLUE;
				hsvThresholdValue[0] = RobotConstants.IP_VAL_MIN_BLUE;
				hsvThresholdValue[1] = RobotConstants.IP_VAL_MAX_BLUE;

				break;
		}

		hsvThreshold(blurOutput,
				hsvThresholdHue,
				hsvThresholdSaturation,
				hsvThresholdValue,
				hsvThresholdOutput[pipelinePass]);

		RobotLog.dd(TAG, "Step HSV_Threshold0 completed");

		// Step CV_erode0:
		Mat cvErodeKernel = new Mat();
		Point cvErodeAnchor = new Point(0, 0);
		double cvErodeIterations = 4.0;
		int cvErodeBordertype = Core.BORDER_CONSTANT;
		Scalar cvErodeBordervalue = new Scalar(-1);
		cvErode(hsvThresholdOutput[pipelinePass], cvErodeKernel, cvErodeAnchor, cvErodeIterations,
				cvErodeBordertype, cvErodeBordervalue, cvErodeOutput[pipelinePass]);

		RobotLog.dd(TAG, "Step CV_erode0 completed");

		// Step CV_dilate0:
		double cvDilateIterations = 3.0;
		int cvDilateBordertype = Core.BORDER_CONSTANT;
		cvDilate(cvErodeOutput[pipelinePass], cvDilateKernel, cvDilateAnchor,
				cvDilateIterations, cvDilateBordertype, cvDilateBorderValue, cvDilateOutput[pipelinePass]);

		RobotLog.dd(TAG, "Step CV_dilate0 completed");

		// Step Find_Contours0:
		boolean findContoursExternalOnly = false;
		//noinspection ConstantConditions
		findContours(cvDilateOutput[pipelinePass], findContoursExternalOnly, findContoursOutput[pipelinePass]);
		if(logging) RobotLog.dd(TAG, "Found %d unfiltered contours", findContoursOutput[pipelinePass].size());

		RobotLog.dd(TAG, "Step Find_Contours0 completed");

		// Step Convex_Hulls0:
		convexHulls(findContoursOutput[pipelinePass], convexHullsOutput[pipelinePass]);
		if(logging) RobotLog.dd(TAG, "Found %d convex contours", findContoursOutput[pipelinePass].size());

		RobotLog.dd(TAG, "Step Convex_Hulls0 completed");

		// Step Filter_Contours0:
		double filterContoursMinArea = 1000.0;
		double filterContoursMinPerimeter = 80.0;
		double filterContoursMinWidth = 20.0;
		double filterContoursMaxWidth = 1000.0;
		double filterContoursMinHeight = 50.0;
		double filterContoursMaxHeight = 1000.0;
		double[] filterContoursSolidity = {0, 100};
		double filterContoursMaxVertices = 20000.0;
		double filterContoursMinVertices = 0.0;
		double filterContoursMinRatio = 0.5;
		double filterContoursMaxRatio = 4.0;
		filterContours(convexHullsOutput[pipelinePass],
				filterContoursMinArea,
				filterContoursMinPerimeter,
				filterContoursMinWidth, filterContoursMaxWidth,
				filterContoursMinHeight, filterContoursMaxHeight,
				filterContoursSolidity, filterContoursMaxVertices, filterContoursMinVertices,
				filterContoursMinRatio, filterContoursMaxRatio, filterContoursOutput[pipelinePass]);

		RobotLog.dd(TAG, "Step Filter_Contours0 completed");
	}

	public Rect roiRect()                               { return roiRect; }
	public Mat roiMat()                                 { return roiMat; }
	public Mat resizeImageOutput()                      { return resizeImageOutput; }
    public Mat blurOutput()                             { return blurOutput; }
	public Mat hsvThresholdOutput(int filterpass)     	{ return hsvThresholdOutput[filterpass];}
	public Mat cvErodeOutput(int filterpass)            { return cvErodeOutput[filterpass]; }
	public Mat cvDilateOutput(int filterpass)       	{ return cvDilateOutput[filterpass]; }
	public ArrayList<MatOfPoint> findContoursOutput(int filterpass)   { return findContoursOutput[filterpass]; }
	public ArrayList<MatOfPoint> convexHullsOutput(int filterpass)    { return convexHullsOutput[filterpass]; }
	public ArrayList<MatOfPoint> filterContoursOutput(int filterpass) { return filterContoursOutput[filterpass]; }


	@SuppressWarnings("SameParameterValue")
	private void resizeImage(Mat input, double width, double height, int interpolation, Mat output)

	{
		RobotLog.ii(TAG, "FfPipeline.resizeImage");
		if(logging) RobotLog.dd(TAG, " resizeImage image inWXH= %dx%d outWXH= %dx%d ", input.cols(),
				input.rows(), output.cols(), output.rows());
		Imgproc.resize(input, output, new Size(width, height), 0.0, 0.0, interpolation);
	}

	enum BlurType
    {
		BOX("Box Blur"), GAUSSIAN("Gaussian Blur"), MEDIAN("Median Filter"),
		BILATERAL("Bilateral Filter");

		private final String label;

		BlurType(String label) {
			this.label = label;
		}

		public static BlurType get(String type)
        {
			if (BILATERAL.label.equals(type))     { return BILATERAL; }
			else if (GAUSSIAN.label.equals(type)) { return GAUSSIAN;  }
			else if (MEDIAN.label.equals(type))   { return MEDIAN;    }
			else                                  { return BOX;       }
		}

		@Override
		public String toString() {
			return this.label;
		}
	}

	private void blur(Mat input, BlurType type, double doubleRadius,
					  Mat output) {
		int radius = (int)(doubleRadius + 0.5);
		int kernelSize;

		if(logging) RobotLog.dd(TAG, " blur image inWXH= %dx%d outWXH= %dx%d", input.cols(),
				input.rows(), output.cols(), output.rows());

		switch(type)
        {
			case BOX:
				kernelSize = 2 * radius + 1;
				Imgproc.blur(input, output, new Size(kernelSize, kernelSize));
				break;
			case GAUSSIAN:
				kernelSize = 6 * radius + 1;
				Imgproc.GaussianBlur(input,output, new Size(kernelSize, kernelSize), radius);
				break;
			case MEDIAN:
				kernelSize = 2 * radius + 1;
				Imgproc.medianBlur(input, output, kernelSize);
				break;
			case BILATERAL:
				Imgproc.bilateralFilter(input, output, -1, radius, radius);
				break;
		}
	}

	/**
	 * Expands area of lower value in an image.
	 * @param src the Image to erode.
	 * @param kernel the kernel for erosion.
	 * @param anchor the center of the kernel.
	 * @param iterations the number of times to perform the erosion.
	 * @param borderType pixel extrapolation method.
	 * @param borderValue value to be used for a constant border.
	 * @param dst Output Image.
	 */
	private void cvErode(Mat src, Mat kernel, Point anchor, double iterations,
		int borderType, Scalar borderValue, Mat dst) {
		if (kernel == null) {
			kernel = new Mat();
		}
		if (anchor == null) {
			anchor = new Point(-1,-1);
		}
		if (borderValue == null) {
			borderValue = new Scalar(-1);
		}
		Imgproc.erode(src, dst, kernel, anchor, (int)iterations, borderType, borderValue);
	}

	/**
	 * Expands area of higher value in an image.
	 * @param src the Image to dilate.
	 * @param kernel the kernel for dilation.
	 * @param anchor the center of the kernel.
	 * @param iterations the number of times to perform the dilation.
	 * @param borderType pixel extrapolation method.
	 * @param borderValue value to be used for a constant border.
	 * @param dst Output Image.
	 */
	private void cvDilate(Mat src, Mat kernel, Point anchor, double iterations,
	int borderType, Scalar borderValue, Mat dst) {
		RobotLog.ii(TAG, "FfPipeline.cvDilate");
		if (kernel == null) {
			kernel = new Mat();
		}
		if (anchor == null) {
			anchor = new Point(-1,-1);
		}
		if (borderValue == null){
			borderValue = new Scalar(-1);
		}
		Imgproc.dilate(src, dst, kernel, anchor, (int)iterations, borderType, borderValue);
	}

	/**
	 * Segment an image based on hue, saturation, and value ranges.
	 *
	 * @param input The image on which to perform the HSL threshold.
	 * @param hue The min and max hue
	 * @param sat The min and max saturation
	 * @param val The min and max value
	 * @param out The image in which to store the output.
	 */
	private void hsvThreshold(Mat input, double[] hue, double[] sat, double[] val, Mat out)
	{
		Imgproc.cvtColor(input, out, Imgproc.COLOR_RGB2HSV);
		Core.inRange(out, new Scalar(hue[0], sat[0], val[0]),
			new Scalar(hue[1], sat[1], val[1]), out);
	}

	private void findContours(Mat input, boolean externalOnly, List<MatOfPoint> contours)
	{
		Mat hierarchy = new Mat();
		contours.clear();
		int mode;
		if (externalOnly) {
			mode = Imgproc.RETR_EXTERNAL;
		}
		else {
			mode = Imgproc.RETR_LIST;
		}
		int method = Imgproc.CHAIN_APPROX_SIMPLE;
		Imgproc.findContours(input, contours, hierarchy, mode, method);
	}

	/**
	 * Compute the convex hulls of contours.
	 * @param inputContours The contours on which to perform the operation.
	 * @param outputContours The contours where the output will be stored.
	 */
	private void convexHulls(List<MatOfPoint> inputContours, ArrayList<MatOfPoint> outputContours)
	{
		final MatOfInt hull = new MatOfInt();
		outputContours.clear();
		for (int i = 0; i < inputContours.size(); i++)
		{
			final MatOfPoint contour = inputContours.get(i);
			final MatOfPoint mopHull = new MatOfPoint();
			Imgproc.convexHull(contour, hull);
			mopHull.create((int) hull.size().height, 1, CvType.CV_32SC2);
			for (int j = 0; j < hull.size().height; j++)
			{
				int index = (int) hull.get(j, 0)[0];
				double[] point = new double[] {contour.get(index, 0)[0], contour.get(index, 0)[1]};
				mopHull.put(j, 0, point);
			}
			outputContours.add(mopHull);
		}
	}


	/**
	 * Filters out contours that do not meet certain criteria.
	 * @param inputContours is the input list of contours
	 * @param output is the the output list of contours
	 * @param minArea is the minimum area of a contour that will be kept
	 * @param minPerimeter is the minimum perimeter of a contour that will be kept
	 * @param minWidth minimum width of a contour
	 *
	 * @param maxWidth maximum width
	 * @param minHeight minimum height
	 * @param maxHeight maximimum height
	 * @param solidity the minimum and maximum solidity of a contour
	 * @param minVertexCount minimum vertex Count of the contours
	 * @param maxVertexCount maximum vertex Count
	 * @param minRatio minimum ratio of width to height
	 * @param maxRatio maximum ratio of width to height
	 */
	private void filterContours(List<MatOfPoint> inputContours, double minArea,
		double minPerimeter, double minWidth, double maxWidth, double minHeight, double
		maxHeight, double[] solidity, double maxVertexCount, double minVertexCount, double
		minRatio, double maxRatio, List<MatOfPoint> output)
	{
		final MatOfInt hull = new MatOfInt();
		output.clear();
		//operation
		for (int i = 0; i < inputContours.size(); i++)
		{
			final MatOfPoint contour = inputContours.get(i);
			final Rect bb = Imgproc.boundingRect(contour);
			if (bb.width < minWidth || bb.width > maxWidth) continue;
			if (bb.height < minHeight || bb.height > maxHeight) continue;
			final double area = Imgproc.contourArea(contour);
			if (area < minArea) continue;
			if (Imgproc.arcLength(new MatOfPoint2f(contour.toArray()), true) < minPerimeter) continue;
			Imgproc.convexHull(contour, hull);
			MatOfPoint mopHull = new MatOfPoint();
			mopHull.create((int) hull.size().height, 1, CvType.CV_32SC2);
			for (int j = 0; j < hull.size().height; j++)
			{
				int index = (int)hull.get(j, 0)[0];
				double[] point = new double[] { contour.get(index, 0)[0], contour.get(index, 0)[1]};
				mopHull.put(j, 0, point);
			}
			final double solid = 100 * area / Imgproc.contourArea(mopHull);
			if (solid < solidity[0] || solid > solidity[1]) continue;
			if (contour.rows() < minVertexCount || contour.rows() > maxVertexCount)	continue;
			final double ratio = bb.width / (double)bb.height;
			if (ratio < minRatio || ratio > maxRatio) continue;
			output.add(contour);
		}
	}
}
