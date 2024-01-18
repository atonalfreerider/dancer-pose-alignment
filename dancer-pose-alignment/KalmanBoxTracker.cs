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
        BoxRatio conv = ConvertBboxToZ(bbox); // X
        kf.StatePre.Set(0, conv.X);
        kf.StatePre.Set(1, conv.Y);
        kf.StatePre.Set(2, conv.Area);
        kf.StatePre.Set(3, conv.Ratio);

        kf.StatePost.Set(0, conv.X);
        kf.StatePost.Set(1, conv.Y);
        kf.StatePost.Set(2, conv.Area);
        kf.StatePost.Set(3, conv.Ratio);
    }

    public void Correct(PoseBoundingBox bbox)
    {
        BoxRatio conv = ConvertBboxToZ(bbox);
        kf.Correct(ToMat(conv));
    }

    public Rectangle Predict()
    {
        Mat prediction = kf.Predict();
        float[] vec = ToVec(prediction);
        return ConvertXToBbox(new BoxRatio(Rnd(vec[0]), Rnd(vec[1]), Rnd(vec[2]), vec[3]));
    }

    // REFERENCE
    static BoxRatio ConvertBboxToZ(PoseBoundingBox bbox)
    {
        int area = bbox.Bounds.Width * bbox.Bounds.Height;
        float ratio = bbox.Bounds.Width / (float) bbox.Bounds.Height;

        return new BoxRatio(bbox.Bounds.X, bbox.Bounds.Y, area, ratio);
    }

    struct BoxRatio(int x, int y, int area, float ratio)
    {
        public readonly int X = x;
        public readonly int Y = y;
        public readonly int Area = area;
        public readonly float Ratio = ratio;
    }

    static Rectangle ConvertXToBbox(BoxRatio boxRatio)
    {
        int w = Rnd(MathF.Sqrt(boxRatio.Area * boxRatio.Ratio)); // w * h * (w / h) = w * w
        int h = Rnd(boxRatio.Area / (float)w); // w * h / w = h
        int x1 = boxRatio.X;
        int y1 = boxRatio.Y;
        int x2 = boxRatio.X + w;
        int y2 = boxRatio.Y + h;

        Rectangle rectangle = new()
        {
            Left = x1,
            Top = y1,
            Right = x2,
            Bottom = y2,
            Width = w,
            Height = h,
            X = boxRatio.X,
            Y = boxRatio.Y
        };
        return rectangle;
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

    static Mat ToMat(BoxRatio vector)
    {
        Mat mat = new(4, 1, MatType.CV_32FC1);
        mat.Set(0, 0, vector.X);
        mat.Set(1, 0, vector.Y);
        mat.Set(2, 0, vector.Area);
        mat.Set(3, 0, vector.Ratio);

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

    static int Rnd(float f)
    {
        return (int)Math.Round(f);
    }
}