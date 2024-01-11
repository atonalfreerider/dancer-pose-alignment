using System.Numerics;
using Compunet.YoloV8.Data;
using LinearAssignment;
using OpenCvSharp;

namespace dancer_pose_alignment;

public class KalmanFilterSort(int maxAge = 1, int minHits = 3, double iouThreshold = 0.3)
{
    readonly List<KalmanBoxTracker> trackers = [];
    int frameCount = 0;
    
    static double[][] IouBatch(List<IPoseBoundingBox> bbTest, double[][] bbGt)
    {
        int n = bbTest.Count;
        int m = bbGt.Length;
        double[][] iouMatrix = new double[n][];
        for (int i = 0; i < n; i++)
        {
            iouMatrix[i] = new double[m];
        }

        for (int i = 0; i < n; i++)
        {
            for (int j = 0; j < m; j++)
            {
                double[] bbTestBox = 
                [
                    bbTest[i].Bounds.Left,
                    bbTest[i].Bounds.Top, 
                    bbTest[i].Bounds.Right,
                    bbTest[i].Bounds.Bottom
                ];
                double[] bbGtBox = bbGt[j];

                double xx1 = Math.Max(bbTestBox[0], bbGtBox[0]);
                double yy1 = Math.Max(bbTestBox[1], bbGtBox[1]);
                double xx2 = Math.Min(bbTestBox[2], bbGtBox[2]);
                double yy2 = Math.Min(bbTestBox[3], bbGtBox[3]);

                double w = Math.Max(0.0, xx2 - xx1);
                double h = Math.Max(0.0, yy2 - yy1);
                double wh = w * h;

                double o = wh / ((bbTestBox[2] - bbTestBox[0]) * (bbTestBox[3] - bbTestBox[1]) +
                    (bbGtBox[2] - bbGtBox[0]) * (bbGtBox[3] - bbGtBox[1]) - wh);

                iouMatrix[i][j] = o;
            }
        }

        return iouMatrix;
    }

    public class KalmanBoxTracker
    {
        static int count = 0;
        readonly KalmanFilter kf;
        
        public readonly int Id;
        int detClass;
        
        public int TimeSinceUpdate = 0;
        public int Hits = 0;
        public int HitStreak = 0;
        public int Age = 0;
        
        readonly List<double[]> history = [];
        readonly List<Tuple<double, double>> centroidArr = [];
        readonly List<IPoseBoundingBox> bboxHistory = [];
        
        public List<Vector3> LastKeypoints
        {
            get
            {
                return bboxHistory[^1].Keypoints.Select(kp => new Vector3(kp.Point.X, kp.Point.Y, kp.Confidence)).ToList();
            }
        }

        public KalmanBoxTracker(IPoseBoundingBox bbox)
        {
            kf = new KalmanFilter(7, 4);
            kf.TransitionMatrix = ToMat(new double[,] // F
            {
                { 1, 0, 0, 0, 1, 0, 0 },
                { 0, 1, 0, 0, 0, 1, 0 },
                { 0, 0, 1, 0, 0, 0, 1 },
                { 0, 0, 0, 1, 0, 0, 0 },
                { 0, 0, 0, 0, 1, 0, 0 },
                { 0, 0, 0, 0, 0, 1, 0 },
                { 0, 0, 0, 0, 0, 0, 1 }
            });
            kf.MeasurementMatrix = ToMat(new double[,] // H
            {
                { 1, 0, 0, 0, 0, 0, 0 },
                { 0, 1, 0, 0, 0, 0, 0 },
                { 0, 0, 1, 0, 0, 0, 0 },
                { 0, 0, 0, 1, 0, 0, 0 }
            });

            kf.MeasurementNoiseCov[2, kf.MeasurementNoiseCov.Rows, 2, kf.MeasurementNoiseCov.Cols] *= 10.0; // R

            kf.ErrorCovPre[4, kf.ErrorCovPre.Rows, 4, kf.ErrorCovPre.Cols] *= 1000.0; // P
            kf.ErrorCovPre *= 10.0; // P
            kf.ProcessNoiseCov *= 0.5; // Q
            kf.ProcessNoiseCov[4, kf.ProcessNoiseCov.Rows, 4, kf.ProcessNoiseCov.Cols] *= 0.5; // Q

            // initialize state
            double[] conv = ConvertBboxToZ(bbox); // X
            kf.StatePre.Set(0, conv[0]);
            kf.StatePre.Set(1, conv[1]);
            kf.StatePre.Set(2, conv[2]);
            kf.StatePre.Set(3, conv[3]);
            
            kf.StatePost.Set(0, conv[0]);
            kf.StatePost.Set(1, conv[1]);
            kf.StatePost.Set(2, conv[2]);
            kf.StatePost.Set(3, conv[3]);
            
            // reset global state
            Id = count;
            count++;
            detClass = bbox.Class.Id;
            
            double cx = bbox.Bounds.X;
            double cy = bbox.Bounds.Y;
            centroidArr.Add(new Tuple<double, double>(cx, cy));
            bboxHistory.Add(bbox);
        }

        public void Correct(IPoseBoundingBox bbox)
        {
            TimeSinceUpdate = 0;
            history.Clear();
            Hits++;
            HitStreak++;
            kf.Correct(ToMat(ConvertBboxToZ(bbox)));
            detClass = bbox.Class.Id;
            double cx = bbox.Bounds.X;
            double cy = bbox.Bounds.Y;
            centroidArr.Add(new Tuple<double, double>(cx, cy));
            bboxHistory.Add(bbox);
        }

        public double[] Predict()
        {
            if (kf.StatePre.Get<double>(6) + kf.StatePre.Get<double>(2) <= 0)
            {
                kf.StatePre.Set(6, 0.0); // X
            }

            kf.Predict();
            Age++;

            if (TimeSinceUpdate > 0)
            {
                HitStreak = 0;
            }

            TimeSinceUpdate++;

            history.Add(ConvertXToBbox(ToVec(kf.StatePost)));
            return history[^1];
        }

        public double[] GetState()
        {
            double[] detClassArray = [detClass];
            double[] uDotArray = [kf.StatePost.Get<double>(4)];
            double[] vDotArray = [kf.StatePost.Get<double>(5)];
            double[] sDotArray = [kf.StatePost.Get<double>(6)];

            return new[]
                {
                    // bbox x, y, s, r
                    kf.StatePost.Get<double>(0),
                    kf.StatePost.Get<double>(1),
                    kf.StatePost.Get<double>(2), 
                    kf.StatePost.Get<double>(3)
                }
                .Concatenate(detClassArray).Concatenate(uDotArray)
                .Concatenate(vDotArray).Concatenate(sDotArray);
        }
        
        static double[] ConvertBboxToZ(IPoseBoundingBox bbox)
        {
            double s = bbox.Bounds.Width * bbox.Bounds.Height;
            double r = bbox.Bounds.Width / bbox.Bounds.Height;

            return [bbox.Bounds.X, bbox.Bounds.Y, s, r];
        }

        static double[] ConvertXToBbox(double[] x, double? score = null)
        {
            double w = Math.Sqrt(x[2] * x[3]);
            double h = x[2] / w;
            double x1 = x[0] - w / 2.0;
            double y1 = x[1] - h / 2.0;
            double x2 = x[0] + w / 2.0;
            double y2 = x[1] + h / 2.0;

            if (score == null)
            {
                return [x1, y1, x2, y2];
            }

            return [x1, y1, x2, y2, score.Value];
        }
    }

    public List<KalmanBoxTracker> GetTrackers()
    {
        return trackers;
    }

    public double[][] Update(List<IPoseBoundingBox> dets)
    {
        frameCount++;
        int n = trackers.Count;
        int m = dets.Count;
        double[][] trks = new double[n][];
        List<int> toDel = [];
        List<double[]> ret = [];
        
        for (int i = 0; i < n; i++)
        {
            double[] pos = trackers[i].Predict();
            trks[i] = [pos[0], pos[1], pos[2], pos[3], 0.0, 0.0];
            if (double.IsNaN(pos[0]) || double.IsNaN(pos[1]))
            {
                toDel.Add(i);
            }
        }

        foreach (int t in toDel)
        {
            trackers.RemoveAt(t);
        }

        int[] matched;
        int[] unmatchedDets;
        int[] unmatchedTrks;

        AssociateDetectionsToTrackers(
            dets,
            trks,
            iouThreshold,
            out matched,
            out unmatchedDets,
            out unmatchedTrks);

        for (int i = 0; i < matched.Length; i++)
        {
            trackers[matched[i]].Correct(dets[i]);
        }

        foreach (int i in unmatchedDets)
        {
            KalmanBoxTracker trk = new(dets[i]);
            trackers.Add(trk);
        }

        int ii = trackers.Count;

        for (int i = ii - 1; i >= 0; i--)
        {
            double[] d = trackers[i].GetState();
            if (trackers[i].TimeSinceUpdate < 1 && (trackers[i].HitStreak >= minHits || frameCount <= minHits))
            {
                d[4] = trackers[i].Id + 1;
                ret.Add(d);
            }

            ii--;

            if (trackers[i].TimeSinceUpdate > maxAge)
            {
                trackers.RemoveAt(i);
            }
        }

        if (ret.Count > 0)
        {
            return ret.ToArray();
        }

        return Array.Empty<double[]>();
    }

    static void AssociateDetectionsToTrackers(
        List<IPoseBoundingBox> detections,
        double[][] trackers,
        double iouThreshold,
        out int[] matches,
        out int[] unmatchedDetections,
        out int[] unmatchedTrackers)
    {
        if (trackers.Length == 0)
        {
            matches = Array.Empty<int>();
            unmatchedDetections = Enumerable.Range(0, detections.Count).ToArray();
            unmatchedTrackers = Array.Empty<int>();
            return;
        }

        double[][] iouMatrix = IouBatch(detections, trackers);

        if (Math.Min(iouMatrix.GetLength(0), iouMatrix.GetLength(1)) > 0)
        {
            int[,] a = new int[iouMatrix.GetLength(0), iouMatrix.GetLength(1)];

            for (int i = 0; i < iouMatrix.GetLength(0); i++)
            {
                for (int j = 0; j < iouMatrix.GetLength(1); j++)
                {
                    a[i, j] = iouMatrix[i][j] > iouThreshold ? 1 : 0;
                }
            }

            int[] rowSums = new int[iouMatrix.GetLength(0)];
            int[] colSums = new int[iouMatrix.GetLength(1)];

            for (int i = 0; i < iouMatrix.GetLength(0); i++)
            {
                for (int j = 0; j < iouMatrix.GetLength(1); j++)
                {
                    rowSums[i] += a[i, j];
                    colSums[j] += a[i, j];
                }
            }

            bool validAssignment = rowSums.Max() == 1 && colSums.Max() == 1;

            int[,] matchedIndices;

            if (validAssignment)
            {
                matchedIndices = new int[iouMatrix.GetLength(0), iouMatrix.GetLength(1)];

                for (int i = 0; i < iouMatrix.GetLength(0); i++)
                {
                    for (int j = 0; j < iouMatrix.GetLength(1); j++)
                    {
                        if (a[i, j] == 1)
                        {
                            matchedIndices[i, 0] = i;
                            matchedIndices[i, 1] = j;
                        }
                    }
                }
            }
            else
            {
                // invert iouMatrix
                for (int i = 0; i < iouMatrix.GetLength(0); i++)
                {
                    for (int j = 0; j < iouMatrix.GetLength(1); j++)
                    {
                        iouMatrix[i][j] = -iouMatrix[i][j];
                    }
                }

                Assignment assignment = Solver.Solve(ToMat(iouMatrix));
                matchedIndices = new int[assignment.RowAssignment.Length, assignment.ColumnAssignment.Length];
                for (int i = 0; i < assignment.ColumnAssignment.Length; i++)
                {
                    for(int j = 0; j < assignment.RowAssignment.Length; j++)
                    {
                        matchedIndices[i, j] = assignment.ColumnAssignment[i];
                    }
                }
            }

            List<int> unmatchedDetectionsList = [];
            List<int> unmatchedTrackersList = [];
            List<int> matchesList = [];

            for (int i = 0; i < detections.Count; i++)
            {
                unmatchedDetectionsList.Add(i);
            }

            for (int i = 0; i < trackers.Length; i++)
            {
                unmatchedTrackersList.Add(i);
            }

            for (int i = 0; i < matchedIndices.GetLength(0); i++)
            {
                int detectionIndex = matchedIndices[i, 0];
                int trackerIndex = matchedIndices[i, 1];

                if (iouMatrix[detectionIndex][trackerIndex] < iouThreshold)
                {
                    unmatchedDetectionsList.Add(detectionIndex);
                    unmatchedTrackersList.Add(trackerIndex);
                }
                else
                {
                    matchesList.Add(detectionIndex);
                }
            }

            matches = matchesList.ToArray();
            unmatchedDetections = unmatchedDetectionsList.ToArray();
            unmatchedTrackers = unmatchedTrackersList.ToArray();
        }
        else
        {
            matches = Array.Empty<int>();
            unmatchedDetections = Enumerable.Range(0, detections.Count).ToArray();
            unmatchedTrackers = Enumerable.Range(0, trackers.Length).ToArray();
        }
    }

    static Mat ToMat(double[,] array)
    {
        int rows = array.GetLength(0);
        int cols = array.GetLength(1);
        Mat mat = new(rows, cols, MatType.CV_64FC1);
        for (int i = 0; i < rows; i++)
        {
            for (int j = 0; j < cols; j++)
            {
                mat.Set(i, j, array[i, j]);
            }
        }

        return mat;
    }

    static Mat ToMat(double[] vector)
    {
        int rows = vector.Length;
        Mat mat = new(rows, 1, MatType.CV_64FC1);
        for (int i = 0; i < rows; i++)
        {
            mat.Set(i, 0, vector[i]);
        }

        return mat;
    }
    
    static double[] ToVec(Mat mat){
        int rows = mat.Rows;
        double[] vector = new double[rows];
        for (int i = 0; i < rows; i++)
        {
            vector[i] = mat.At<double>(i, 0);
        }

        return vector;
    }
    
    static double[,] ToMat(double[][] array)
    {
        int rows = array.Length;
        int cols = array[0].Length;
        double[,] mat = new double[rows, cols];
        for (int i = 0; i < rows; i++)
        {
            for (int j = 0; j < cols; j++)
            {
                mat[i, j] = array[i][j];
            }
        }

        return mat;
    }
}

public static class Extensions
{
    public static T[] Concatenate<T>(this T[] first, T[] second)
    {
        if (first == null)
        {
            throw new ArgumentNullException(nameof(first));
        }

        if (second == null)
        {
            throw new ArgumentNullException(nameof(second));
        }

        T[] result = new T[first.Length + second.Length];
        first.CopyTo(result, 0);
        second.CopyTo(result, first.Length);
        return result;
    }
}