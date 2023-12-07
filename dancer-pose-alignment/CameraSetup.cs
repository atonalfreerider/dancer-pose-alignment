using System.Numerics;

namespace dancer_pose_alignment;

public class CameraSetup
{
    public readonly List<Vector3> PositionsPerFrame = new();
    public readonly List<Quaternion> RotationsPerFrame = new();
    public Vector2 Size;
    public float FocalLength = .2f;

    const float PixelToMeter = 0.000264583f;

    public Vector3 Forward(int frame) => Vector3.Transform(
        new Vector3(0, 0, 1),
        RotationsPerFrame[frame]);

    public List<List<Vector3>> LeadProjectionsPerFrame = new();
    public List<List<Vector3>> FollowProjectionsPerFrame = new();

    readonly List<List<Vector3>> LeadPoseAndConfidencePerFrame = new();
    readonly List<List<Vector3>> FollowPoseAndConfidencePerFrame = new();

    public void Project(int frameNumber)
    {
        List<Vector3> flattenedLead = LeadPoseAndConfidencePerFrame[frameNumber].Select(x => x with { Z = 0 }).ToList();
        List<Vector3> flattenedFollow = FollowPoseAndConfidencePerFrame[frameNumber].Select(x => x with { Z = 0 }).ToList();
        
        List<Vector3> leadProjectionsAtThisFrame = Adjusted(flattenedLead, frameNumber);
        LeadProjectionsPerFrame.Add(leadProjectionsAtThisFrame);
        
        List<Vector3> followProjectionsAtThisFrame = Adjusted(flattenedFollow, frameNumber);
        FollowProjectionsPerFrame.Add(followProjectionsAtThisFrame);
    }

    public float Error(List<Vector3> merged3DPoseLead, List<Vector3> merged3DPoseFollow)
    {
        float error = 0;
        
        //TODO
        // cast rays directly from the camera position to the 3D pose position for each list, and then find the
        // intersections on the plane at the focal length distance in front of the camera.
        // sum the distance from the 2D pose to the intersection points to find the error
        


        return error;
    }

    public void AddPosesAndRecenterAndScaleToCamera(
        IEnumerable<Vector3> lead2DPixelsAndConfidence,
        IEnumerable<Vector3> follow2PixelsAndConfidence)
    {
        List<Vector3> leadRecenteredAndRescaled = lead2DPixelsAndConfidence.Select(vec => new Vector3(
                (vec.X - Size.X / 2) * PixelToMeter,
                -(vec.Y - Size.Y / 2) * PixelToMeter, // flip
                vec.Z)) // keep the confidence
            .ToList();
        LeadPoseAndConfidencePerFrame.Add(leadRecenteredAndRescaled);

        List<Vector3> followRecenteredAndRescaled = follow2PixelsAndConfidence.Select(vec => new Vector3(
                (vec.X - Size.X / 2) * PixelToMeter,
                -(vec.Y - Size.Y / 2) * PixelToMeter, // flip
                vec.Z)) // keep the confidence
            .ToList();

        FollowPoseAndConfidencePerFrame.Add(followRecenteredAndRescaled);
    }

    List<Vector3> Adjusted(IEnumerable<Vector3> keypoints, int frame)
    {
        // Translate keypoints to the camera center
        Vector3 cameraCenter = PositionsPerFrame[frame];
        List<Vector3> adjustedKeypoints = keypoints.Select(t => cameraCenter + t).ToList();

        // Rotate keypoints around the camera center by the camera's rotation quaternion
        Quaternion rotation = RotationsPerFrame[frame];
        for (int i = 0; i < adjustedKeypoints.Count; i++)
        {
            adjustedKeypoints[i] = Vector3.Transform(adjustedKeypoints[i] - cameraCenter, rotation) + cameraCenter;
        }
        
        // Translate keypoints to the camera's focal length
        adjustedKeypoints = adjustedKeypoints.Select(t => t + Forward(frame) * FocalLength).ToList();

        return adjustedKeypoints;
    }
}