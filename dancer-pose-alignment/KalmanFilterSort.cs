using Compunet.YoloV8.Data;
using LinearAssignment;

namespace dancer_pose_alignment;

public class KalmanFilterSort(int maxAge = 1, int minHits = 3, float iouThreshold = 0.3f)
{
    readonly List<KalmanBoxTracker> trackers = [];
    int frameCount;

    public List<KalmanBoxTracker> Update(List<IPoseBoundingBox> detections)
    {
        frameCount++;
        int n = trackers.Count;
        float[][] trackerValues = new float[n][];
        List<int> toDelete = [];

        // populate current tracker values, and eliminate NaN values
        for (int i = 0; i < n; i++)
        {
            float[] pos = trackers[i].Predict();
            trackerValues[i] = [pos[0], pos[1], pos[2], pos[3], 0, 0];
            if (float.IsNaN(pos[0]) || float.IsNaN(pos[1]))
            {
                toDelete.Add(i);
            }
        }

        foreach (int t in toDelete)
        {
            trackers.RemoveAt(t);
        }

        (int[] matchedDetections, int[] unmatchedDetections) = AssociateDetectionsToTrackers(
            detections,
            trackerValues,
            iouThreshold);

        for (int i = 0; i < matchedDetections.Length; i++)
        {
            trackers[matchedDetections[i]].Correct(detections[i]);
        }

        foreach (int i in unmatchedDetections)
        {
            KalmanBoxTracker trk = new(detections[i]);
            trackers.Add(trk);
        }

        int ii = trackers.Count;

        for (int i = ii - 1; i >= 0; i--)
        {
            float[] trackerState = trackers[i].GetState();
            if (trackers[i].TimeSinceUpdate < 1 &&
                (trackers[i].HitStreak >= minHits ||
                 frameCount <= minHits))
            {
                trackerState[4] = trackers[i].Id + 1;
            }

            ii--;

            if (trackers[i].TimeSinceUpdate > maxAge)
            {
                trackers.RemoveAt(i);
            }
        }

        return trackers;
    }

    static (int[] MatchedIndices, int[] UnmatchedDetections) AssociateDetectionsToTrackers(
        List<IPoseBoundingBox> detections,
        float[][] trackers,
        double iouThreshold = 0.3)
    {
        if (trackers.Length == 0)
        {
            return (
                Array.Empty<int>(), 
                Enumerable.Range(0, detections.Count).ToArray());
        }
        
        double[,] iouMatrix = CalculateIoUMatrix(detections, trackers);
        int[] matchedIndices = Array.Empty<int>();
        
        if (IsJaggedArrayNonEmpty(trackers))
        {
            // there is an intersection, find matching matrix
            int[,] matchingMatrix = CalculateMatchingMatrix(iouMatrix, iouThreshold);
            if (IsMaximumSumOne(matchingMatrix))
            {
                matchedIndices = GetMatchedIndices(matchingMatrix);
            }
            else
            {
                Assignment assignment = Solver.Solve(Negated(iouMatrix));
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
        }
        else
        {
            matchedIndices = new[,] { { 0, 0 } }.Cast<int>().ToArray();
        }
        
        int[] unmatchedDetections = FindUnmatchedDetections(detections.Count, matchedIndices);

        return (matchedIndices, unmatchedDetections);
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

    /// <summary>
    /// Inverse over Union Matrix
    ///
    /// Used to find overlap between prediction and observation
    /// </summary>
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
                matchingMatrix[i, j] = iouMatrix[i, j] > iouThreshold ? 1 : 0;
            }
        }

        return matchingMatrix;
    }

    static bool IsJaggedArrayNonEmpty(float[][] jaggedArray)
    {
        if (jaggedArray.Length > 0)
        {
            foreach (float[] row in jaggedArray)
            {
                if (row.Length > 0)
                {
                    return true; // Found at least one non-empty row
                }
            }
        }

        return false; // Jagged array is empty or all rows are empty
    }

    static double[,] Negated(double[,] iouMatrix)
    {
        double[,] negatedIoUMatrix = new double[iouMatrix.GetLength(0), iouMatrix.GetLength(1)];

        for (int i = 0; i < iouMatrix.GetLength(0); i++)
        {
            for (int j = 0; j < iouMatrix.GetLength(1); j++)
            {
                negatedIoUMatrix[i, j] = -iouMatrix[i, j];
            }
        }

        return negatedIoUMatrix;
    }
    
    static bool IsMaximumSumOne(int[,] matrix)
    {
        int rows = matrix.GetLength(0);
        int cols = matrix.GetLength(1);

        for (int i = 0; i < rows; i++)
        {
            if (GetRowSum(matrix, i) > 1) return false;
        }

        for (int j = 0; j < cols; j++)
        {
            if (GetColumnSum(matrix, j) > 1) return false;
        }

        return true;
    }

    static int[] GetMatchedIndices(int[,] matrix)
    {
        List<int> indices = [];
        int rows = matrix.GetLength(0);
        int cols = matrix.GetLength(1);

        for (int i = 0; i < rows; i++)
        {
            for (int j = 0; j < cols; j++)
            {
                if (matrix[i, j] == 1)
                {
                    indices.Add(i); // Add row index
                    indices.Add(j); // Add column index
                }
            }
        }

        return indices.ToArray();
    }

    static int GetRowSum(int[,] matrix, int row)
    {
        int sum = 0;
        int cols = matrix.GetLength(1);

        for (int j = 0; j < cols; j++)
        {
            sum += matrix[row, j];
        }

        return sum;
    }

    static int GetColumnSum(int[,] matrix, int col)
    {
        int sum = 0;
        int rows = matrix.GetLength(0);

        for (int i = 0; i < rows; i++)
        {
            sum += matrix[i, col];
        }

        return sum;
    }
}