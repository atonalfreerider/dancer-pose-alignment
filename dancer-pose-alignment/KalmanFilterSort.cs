using Compunet.YoloV8.Data;

namespace dancer_pose_alignment;

public class KalmanFilterSort(int maxAge = 1, int minHits = 3, float iouThreshold = 0.3f)
{
    readonly List<KalmanBoxTracker> trackers = [];

    public List<KalmanBoxTracker> Update(List<IPoseBoundingBox> detections)
    {
        int n = trackers.Count;
        float[][] trackerValues = new float[n][];
        
        // populate current tracker values, and eliminate NaN values
        for (int i = 0; i < n; i++)
        {
            float[] pos = trackers[i].Predict();
            trackerValues[i] = [pos[0], pos[1], pos[2], pos[3]];
        }

        Dictionary<int, int> iouMatrix = CalculateIoUMatrix(
            detections,
            trackerValues,
            iouThreshold);
        
        List<KalmanBoxTracker> updatedTrackers = [];

        foreach ((int detInd, int trackInd) in iouMatrix)
        {
            if (trackInd > -1)
            {
                trackers[trackInd].Correct(detections[detInd]);
                updatedTrackers.Add(trackers[trackInd]);
            }
            else
            {
                KalmanBoxTracker trk = new(detections[detInd]);
                trackers.Add(trk);
                updatedTrackers.Add(trk);
            }
        }

        return updatedTrackers;
    }

    /// <summary>
    /// Inverse over Union Matrix
    ///
    /// Used to find overlap between prediction and observation. Each row represents the overlap of a detection with
    /// each tracker
    /// </summary>
    static Dictionary<int, int> CalculateIoUMatrix(
        List<IPoseBoundingBox> detections, 
        float[][] trackers,
        double iouThreshold)
    {
        int numDetections = detections.Count;
        int numTrackers = trackers.GetLength(0);

        Dictionary<int, int> detectorsAndMatchedTracker = [];
        for (int i = 0; i < numDetections; i++)
        {
            int highestTrackerIouIndex = -1;
            double highestIou = iouThreshold;
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
                if (iou > highestIou)
                {
                    highestTrackerIouIndex = j;
                    highestIou = iou;
                }
            }

            if (!detectorsAndMatchedTracker.ContainsValue(highestTrackerIouIndex))
            {
                detectorsAndMatchedTracker.Add(i, highestTrackerIouIndex);
            }
            else
            {
                detectorsAndMatchedTracker.Add(i, -1);
            }
        }

        return detectorsAndMatchedTracker;
    }
}