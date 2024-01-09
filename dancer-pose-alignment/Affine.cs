using System.Numerics;
using OpenCvSharp;

namespace dancer_pose_alignment;

public class Affine
{
    readonly Dictionary<string, InputArray> previousImages = [];
    readonly Dictionary<string, InputArray> previousTrackingPoints = [];

    public void Init(Mat frameMat, string videoPath)
    {
        (InputArray? toTrack, InputOutputArray? pointsToTrack) = ImgAndPts(frameMat);

        previousImages[videoPath] = toTrack;
        previousTrackingPoints[videoPath] = pointsToTrack.GetMat();
    }

    public Vector3 GetAffine(Mat frameMat, string videoPath)
    {
        (InputArray? toTrack, InputOutputArray? pointsToTrack) = ImgAndPts(frameMat);

        OutputArray status = new Mat();
        OutputArray err = new Mat();
        Cv2.CalcOpticalFlowPyrLK(
            previousImages[videoPath], // previous img
            toTrack, // current img
            previousTrackingPoints[videoPath],
            pointsToTrack,
            status,
            err);

        Mat transform =
            Cv2.EstimateAffinePartial2D(previousTrackingPoints[videoPath], pointsToTrack.GetMat());

        // Extract translation
        double dx = transform.At<double>(0, 2);
        double dy = transform.At<double>(1, 2);

        // Extract rotation angle
        double da = Math.Atan2(transform.At<double>(1, 0), transform.At<double>(0, 0));

        previousImages[videoPath] = toTrack;
        previousTrackingPoints[videoPath] = pointsToTrack.GetMat();

        return new Vector3((float)dx, (float)dy, (float)da);
    }

    static Tuple<InputArray, InputOutputArray> ImgAndPts(Mat frameMat)
    {
        // optical flow for camera motion
        InputArray toGray = InputArray.Create(frameMat);
        OutputArray grayImg = new Mat();
        Cv2.CvtColor(toGray, grayImg, ColorConversionCodes.BGR2GRAY);
        InputArray toTrack = InputArray.Create(grayImg.GetMat());
        Point2f[] points = Cv2.GoodFeaturesToTrack(
            toTrack,
            100,
            0.01,
            10,
            null,
            3,
            false,
            0.04);

        Mat pointsMat = new(points.Length, 1, MatType.CV_32FC2);
        pointsMat.SetArray(points);

        InputOutputArray pointsToTrack = InputOutputArray.Create(pointsMat);

        return new Tuple<InputArray, InputOutputArray>(toTrack, pointsToTrack);
    }
}