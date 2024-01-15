using System.Numerics;

using OpenCvSharp;

namespace dancer_pose_alignment;

public class KalmanBoxTracker
{
    readonly KalmanFilter kf;

    public KalmanBoxTracker()
    {
        kf = new KalmanFilter(7, 4);

        kf.TransitionMatrix = ToMat(new float[,] // F
        {
            { 1, 0, 0, 0, 1, 0, 0 },
            { 0, 1, 0, 0, 0, 1, 0 },
            { 0, 0, 1, 0, 0, 0, 1 },
            { 0, 0, 0, 1, 0, 0, 0 },
            { 0, 0, 0, 0, 1, 0, 0 },
            { 0, 0, 0, 0, 0, 1, 0 },
            { 0, 0, 0, 0, 0, 0, 1 }
        });

        kf.MeasurementMatrix = ToMat(new float[,] // H
        {
            { 1, 0, 0, 0, 0, 0, 0 },
            { 0, 1, 0, 0, 0, 0, 0 },
            { 0, 0, 1, 0, 0, 0, 0 },
            { 0, 0, 0, 1, 0, 0, 0 }
        });

        kf.MeasurementNoiseCov[
            2, kf.MeasurementNoiseCov.Rows,
            2, kf.MeasurementNoiseCov.Cols] *= 10.0; // R

        kf.ErrorCovPre[
            4, kf.ErrorCovPre.Rows,
            4, kf.ErrorCovPre.Cols] *= 1000.0; // P

        kf.ErrorCovPre *= 10.0; // P

        kf.ProcessNoiseCov *= 0.5; // Q
        kf.ProcessNoiseCov[
            4, kf.ProcessNoiseCov.Rows,
            4, kf.ProcessNoiseCov.Cols] *= 0.5; // Q
    }

    public void Init(PoseBoundingBox bbox)
    {
        // initialize state
        float[] conv = ConvertBboxToZ(bbox); // X
        kf.StatePre.Set(0, conv[0]);
        kf.StatePre.Set(1, conv[1]);
        kf.StatePre.Set(2, conv[2]);
        kf.StatePre.Set(3, conv[3]);

        kf.StatePost.Set(0, conv[0]);
        kf.StatePost.Set(1, conv[1]);
        kf.StatePost.Set(2, conv[2]);
        kf.StatePost.Set(3, conv[3]);
    }

    public void Correct(PoseBoundingBox bbox)
    {
        kf.Correct(ToMat(ConvertBboxToZ(bbox)));
    }

    public float[] Predict()
    {
        if (kf.StatePre.Get<float>(6) + kf.StatePre.Get<float>(2) <= 0)
        {
            kf.StatePre.Set(6, 0); // X
        }
        
        return ConvertXToBbox(ToVec(kf.Predict()));
    }

    // REFERENCE
    static float[] ConvertBboxToZ(PoseBoundingBox bbox)
    {
        float s = bbox.Bounds.Width * bbox.Bounds.Height;
        float r = bbox.Bounds.Width / bbox.Bounds.Height;

        return [bbox.Bounds.X, bbox.Bounds.Y, s, r];
    }

    static float[] ConvertXToBbox(float[] x, float? score = null)
    {
        float w = MathF.Sqrt(x[2] * x[3]);
        float h = x[2] / w;
        float x1 = x[0] - w / 2;
        float y1 = x[1] - h / 2;
        float x2 = x[0] + w / 2;
        float y2 = x[1] + h / 2;

        if (score == null)
        {
            return [x1, y1, x2, y2];
        }

        return [x1, y1, x2, y2, score.Value];
    }

    static Mat ToMat(float[,] array)
    {
        int rows = array.GetLength(0);
        int cols = array.GetLength(1);
        Mat mat = new(rows, cols, MatType.CV_32FC1);
        for (int i = 0; i < rows; i++)
        {
            for (int j = 0; j < cols; j++)
            {
                mat.Set(i, j, array[i, j]);
            }
        }

        return mat;
    }

    static Mat ToMat(float[] vector)
    {
        int rows = vector.Length;
        Mat mat = new(rows, 1, MatType.CV_32FC1);
        for (int i = 0; i < rows; i++)
        {
            mat.Set(i, 0, vector[i]);
        }

        return mat;
    }

    static float[] ToVec(Mat mat)
    {
        int rows = mat.Rows;
        float[] vector = new float[rows];
        for (int i = 0; i < rows; i++)
        {
            vector[i] = mat.At<float>(i, 0);
        }

        return vector;
    }
}