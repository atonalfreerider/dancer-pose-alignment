using Compunet.YoloV8.Data;

namespace dancer_pose_alignment;

public class KalmanFilterSort(int maxAge = 1, int minHits = 3, float iouThreshold = 0.3f)
{
    public readonly List<KalmanBoxTracker> Trackers = [];
    int frameCount = 0;

    public float[][] Update(List<IPoseBoundingBox> detections)
    {
        frameCount++;
        int n = Trackers.Count;
        int m = detections.Count;
        float[][] trks = new float[n][];
        List<int> toDel = [];
        List<float[]> ret = [];

        for (int i = 0; i < n; i++)
        {
            float[] pos = Trackers[i].Predict();
            trks[i] = [pos[0], pos[1], pos[2], pos[3], 0, 0];
            if (float.IsNaN(pos[0]) || float.IsNaN(pos[1]))
            {
                toDel.Add(i);
            }
        }

        foreach (int t in toDel)
        {
            Trackers.RemoveAt(t);
        }

        (int[] matched, int[] unmatchedDets) = AssociateDetectionsToTrackers(
            detections,
            trks,
            iouThreshold);

        for (int i = 0; i < matched.Length; i++)
        {
            Trackers[matched[i]].Correct(detections[i]);
        }

        foreach (int i in unmatchedDets)
        {
            KalmanBoxTracker trk = new(detections[i]);
            Trackers.Add(trk);
        }

        int ii = Trackers.Count;

        for (int i = ii - 1; i >= 0; i--)
        {
            float[] d = Trackers[i].GetState();
            if (Trackers[i].TimeSinceUpdate < 1 && (Trackers[i].HitStreak >= minHits || frameCount <= minHits))
            {
                d[4] = Trackers[i].Id + 1;
                ret.Add(d);
            }

            ii--;

            if (Trackers[i].TimeSinceUpdate > maxAge)
            {
                Trackers.RemoveAt(i);
            }
        }

        return ret.Count > 0
            ? ret.ToArray()
            : Array.Empty<float[]>();
    }

    static (int[] MatchedIndices, int[] UnmatchedDetections) AssociateDetectionsToTrackers(
        List<IPoseBoundingBox> detections,
        float[][] trackers,
        double iouThreshold = 0.3)
    {
        if (!IsJaggedArrayNonEmpty(trackers))
        {
            return (Array.Empty<int>(), Enumerable.Range(0, detections.Count).ToArray());
        }

        double[,] iouMatrix = CalculateIoUMatrix(detections, trackers);

        if (Math.Min(iouMatrix.GetLength(0), iouMatrix.GetLength(1)) > 0)
        {
            int[,] a = CalculateMatchingMatrix(iouMatrix, iouThreshold);
            int[] matchedIndices;

            if (a.GetLength(0) == 1 && a.GetLength(1) == 1)
            {
                matchedIndices = new int[,] { { 0, 0 } }.Cast<int>().ToArray();
            }
            else
            {
                double[,] negatedIoUMatrix = new double[iouMatrix.GetLength(0), iouMatrix.GetLength(1)];

                for (int i = 0; i < iouMatrix.GetLength(0); i++)
                {
                    for (int j = 0; j < iouMatrix.GetLength(1); j++)
                    {
                        negatedIoUMatrix[i, j] = -iouMatrix[i, j];
                    }
                }

                LinearAssignment.Assignment assignment = LinearAssignment.Solver.Solve(negatedIoUMatrix);
                List<int> matchedIndicesList = [];
                for (int i = 0; i < assignment.RowAssignment.Length; i++)
                {
                    for (int j = 0; j < assignment.ColumnAssignment.Length; j++)
                    {
                        if (assignment.RowAssignment[i] == j)
                        {
                            matchedIndicesList.Add(i);
                        }
                    }
                }

                matchedIndices = matchedIndicesList.ToArray();
            }

            int[] unmatchedDetections = FindUnmatchedDetections(detections.Count, matchedIndices);
            return (matchedIndices, unmatchedDetections);
        }

        return (Array.Empty<int>(), Enumerable.Range(0, detections.Count).ToArray());
    }

    static int[] FindUnmatchedDetections(int numDetections, int[] matchedIndices)
    {
        HashSet<int> matchedSet = [..matchedIndices];
        List<int> unmatchedDetections = [];

        for (int i = 0; i < numDetections; i++)
        {
            if (!matchedSet.Contains(i))
            {
                unmatchedDetections.Add(i);
            }
        }

        return unmatchedDetections.ToArray();
    }

    static double[,] CalculateIoUMatrix(List<IPoseBoundingBox> detections, float[][] trackers)
    {
        int numDetections = detections.Count;
        int numTrackers = trackers.GetLength(0);
        double[,] iouMatrix = new double[numDetections, numTrackers];

        for (int i = 0; i < numDetections; i++)
        {
            for (int j = 0; j < numTrackers; j++)
            {
                // find smallest intersection box
                double xx1 = Math.Max(detections[i].Bounds.Left, trackers[j][0]);
                double yy1 = Math.Max(detections[i].Bounds.Top, trackers[j][1]);
                double xx2 = Math.Min(detections[i].Bounds.Right, trackers[j][2]);
                double yy2 = Math.Min(detections[i].Bounds.Bottom, trackers[j][3]);
                double w = Math.Max(0.0, xx2 - xx1);
                double h = Math.Max(0.0, yy2 - yy1);
                double intersection = w * h;
                double detArea = detections[i].Bounds.Width * detections[i].Bounds.Height;
                double trkArea = (trackers[j][2] - trackers[j][0]) * (trackers[j][3] - trackers[j][1]);
                double union = detArea + trkArea - intersection;
                double iou = intersection / union;
                iouMatrix[i, j] = iou;
            }
        }

        return iouMatrix;
    }

    static int[,] CalculateMatchingMatrix(double[,] iouMatrix, double iouThreshold)
    {
        int numRows = iouMatrix.GetLength(0);
        int numCols = iouMatrix.GetLength(1);
        int[,] matchingMatrix = new int[numRows, numCols];

        for (int i = 0; i < numRows; i++)
        {
            for (int j = 0; j < numCols; j++)
            {
                matchingMatrix[i, j] = (iouMatrix[i, j] > iouThreshold) ? 1 : 0;
            }
        }

        return matchingMatrix;
    }

    static bool IsJaggedArrayNonEmpty(float[][] jaggedArray)
    {
        if (jaggedArray != null && jaggedArray.Length > 0)
        {
            foreach (float[] row in jaggedArray)
            {
                if (row != null && row.Length > 0)
                {
                    return true; // Found at least one non-empty row
                }
            }
        }

        return false; // Jagged array is empty or all rows are empty
    }
}