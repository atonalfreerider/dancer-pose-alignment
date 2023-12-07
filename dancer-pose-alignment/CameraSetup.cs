using System.Numerics;

namespace dancer_pose_alignment;

public class CameraSetup
{
    public readonly List<Vector3> PositionsPerFrame = [];
    public readonly List<Quaternion> RotationsPerFrame = [];
    public Vector2 Size;
    public float FocalLength = .05f;

    const float PixelToMeter = 0.000264583f;

    public Vector3 Forward(int frame) => Vector3.Transform(
        new Vector3(0, 0, 1),
        RotationsPerFrame[frame]);

    public readonly List<List<Vector3>> LeadProjectionsPerFrame = [];
    public readonly List<List<Vector3>> FollowProjectionsPerFrame = [];

    readonly List<List<Vector3>> LeadPoseAndConfidencePerFrame = [];
    readonly List<List<Vector3>> FollowPoseAndConfidencePerFrame = [];

    public void Project(int frameNumber)
    {
        List<Vector3> flattenedLead = LeadPoseAndConfidencePerFrame[frameNumber]
            .Select(x => x with { Z = 0 }).ToList();
        List<Vector3> flattenedFollow = FollowPoseAndConfidencePerFrame[frameNumber]
            .Select(x => x with { Z = 0 }).ToList();

        List<Vector3> leadProjectionsAtThisFrame = Adjusted(flattenedLead, frameNumber);
        LeadProjectionsPerFrame.Add(leadProjectionsAtThisFrame);

        List<Vector3> followProjectionsAtThisFrame = Adjusted(flattenedFollow, frameNumber);
        FollowProjectionsPerFrame.Add(followProjectionsAtThisFrame);
    }

    public float Error(List<Vector3> merged3DPoseLead, List<Vector3> merged3DPoseFollow, int frameNumber)
    {
        float error = 0;

        for (int i = 0; i < merged3DPoseLead.Count; i++)
        {
            Vector3 vector3 = merged3DPoseLead[i];
            Vector3 target = Vector3.Normalize(vector3 - PositionsPerFrame[frameNumber]);
            Vector3 keypoint = Vector3.Normalize(
                LeadProjectionsPerFrame[frameNumber][i] - PositionsPerFrame[frameNumber]);

            // find the angle between the vectors
            error += MathF.Acos(Vector3.Dot(target, keypoint)) * 
                     LeadPoseAndConfidencePerFrame[frameNumber][i].Z; // confidence
        }

        for (int i = 0; i < merged3DPoseFollow.Count; i++)
        {
            Vector3 vector3 = merged3DPoseFollow[i];
            Vector3 target = Vector3.Normalize(vector3 - PositionsPerFrame[frameNumber]);
            Vector3 keypoint = Vector3.Normalize(
                FollowProjectionsPerFrame[frameNumber][i] - PositionsPerFrame[frameNumber]);

            // find the angle between the vectors
            error += MathF.Acos(Vector3.Dot(target, keypoint)) * 
                     FollowPoseAndConfidencePerFrame[frameNumber][i].Z; // confidence
        }


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
        return adjustedKeypoints.Select(t => t + Forward(frame) * FocalLength).ToList();
    }

    public Ray PoseRay(int frameNumber, int jointNumber, bool isLead)
    {
        Ray rayToJoint = new Ray(
            PositionsPerFrame[frameNumber],
            Vector3.Normalize(isLead
                ? LeadProjectionsPerFrame[frameNumber][jointNumber]
                : FollowProjectionsPerFrame[frameNumber][jointNumber]
                  - PositionsPerFrame[frameNumber]));
        return rayToJoint;
    }

    public float JointConfidence(int frameNumber, int jointNumber, bool isLead)
    {
        return isLead
            ? LeadPoseAndConfidencePerFrame[frameNumber][jointNumber].Z
            : FollowPoseAndConfidencePerFrame[frameNumber][jointNumber].Z;
    }

    public List<float> PoseConfidences(int frameNumber, bool isLead)
    {
        return isLead
            ? LeadPoseAndConfidencePerFrame[frameNumber].Select(vec => vec.Z).ToList()
            : FollowPoseAndConfidencePerFrame[frameNumber].Select(vec => vec.Z).ToList();
    }
}