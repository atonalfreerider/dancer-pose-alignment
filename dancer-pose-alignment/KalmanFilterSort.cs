using Compunet.YoloV8.Data;
using LinearAssignment;
using OpenCvSharp;

namespace dancer_pose_alignment;

public class KalmanFilterSort(int maxAge = 1, int minHits = 3, float iouThreshold = 0.3f)
{
    readonly List<KalmanBoxTracker> trackers = [];
    int frameCount = 0;

    static float[][] IouBatch(List<IPoseBoundingBox> bboxTest, float[][] bboxGt)
    {
        int n = bboxTest.Count;
        int m = bboxGt.Length;
        float[][] iouMatrix = new float[n][];
        for (int i = 0; i < n; i++)
        {
            iouMatrix[i] = new float[m];
        }

        for (int i = 0; i < n; i++)
        {
            for (int j = 0; j < m; j++)
            {
                float[] bbTestBox =
                [
                    bboxTest[i].Bounds.Left,
                    bboxTest[i].Bounds.Top,
                    bboxTest[i].Bounds.Right,
                    bboxTest[i].Bounds.Bottom
                ];
                float[] bbGtBox = bboxGt[j];

                float xx1 = Math.Max(bbTestBox[0], bbGtBox[0]);
                float yy1 = Math.Max(bbTestBox[1], bbGtBox[1]);
                float xx2 = Math.Min(bbTestBox[2], bbGtBox[2]);
                float yy2 = Math.Min(bbTestBox[3], bbGtBox[3]);

                float w = MathF.Max(0, xx2 - xx1);
                float h = MathF.Max(0, yy2 - yy1);
                float wh = w * h;

                float o = wh / ((bbTestBox[2] - bbTestBox[0]) * (bbTestBox[3] - bbTestBox[1]) +
                    (bbGtBox[2] - bbGtBox[0]) * (bbGtBox[3] - bbGtBox[1]) - wh);

                iouMatrix[i][j] = o;
            }
        }

        return iouMatrix;
    }

    public List<KalmanBoxTracker> GetTrackers()
    {
        return trackers;
    }

    public float[][] Update(List<IPoseBoundingBox> detections)
    {
        frameCount++;
        int n = trackers.Count;
        int m = detections.Count;
        float[][] trks = new float[n][];
        List<int> toDel = [];
        List<float[]> ret = [];

        for (int i = 0; i < n; i++)
        {
            float[] pos = trackers[i].Predict();
            trks[i] = [pos[0], pos[1], pos[2], pos[3], 0, 0];
            if (float.IsNaN(pos[0]) || float.IsNaN(pos[1]))
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
            detections,
            trks,
            iouThreshold,
            out matched,
            out unmatchedDets,
            out unmatchedTrks);

        for (int i = 0; i < matched.Length; i++)
        {
            trackers[matched[i]].Correct(detections[i]);
        }

        foreach (int i in unmatchedDets)
        {
            KalmanBoxTracker trk = new(detections[i]);
            trackers.Add(trk);
        }

        int ii = trackers.Count;

        for (int i = ii - 1; i >= 0; i--)
        {
            float[] d = trackers[i].GetState();
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

        return Array.Empty<float[]>();
    }

    static void AssociateDetectionsToTrackers(
        List<IPoseBoundingBox> detections,
        float[][] trackers,
        float iouThreshold,
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

        float[][] iouMatrix = IouBatch(detections, trackers);

        if (IsJaggedArrayNonEmpty(iouMatrix))
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
                    for (int j = 0; j < assignment.RowAssignment.Length; j++)
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

    static double[,] ToMat(float[][] array)
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