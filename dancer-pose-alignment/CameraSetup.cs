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

    public Vector3 Up(int frame) => Vector3.Transform(
        new Vector3(0, 1, 0),
        RotationsPerFrame[frame]);

    public Vector3 Right(int frame) => Vector3.Transform(
        new Vector3(1, 0, 0),
        RotationsPerFrame[frame]);

    // these are serialized - leave them public
    public readonly List<List<Vector3>> LeadProjectionsPerFrame = [];
    public readonly List<List<Vector3>> FollowProjectionsPerFrame = [];

    List<List<Vector3>> leadPoseAndConfidencePerFrame;
    List<List<Vector3>> followPoseAndConfidencePerFrame;
    List<List<Vector3>> mirrorLeadPoseAndConfidencePerFrame;
    List<List<Vector3>> mirrorFollowPoseAndConfidencePerFrame;
    
    List<List<Vector3>> recenteredLeadPoseAndConfidencePerFrame;
    List<List<Vector3>> recenteredFollowPoseAndConfidencePerFrame;
    List<List<Vector3>> mirrorRecenteredLeadPoseAndConfidencePerFrame;
    List<List<Vector3>> mirrorRecenteredFollowPoseAndConfidencePerFrame;

    public void Project(int frameNumber)
    {
        List<Vector3> flattenedLead = recenteredLeadPoseAndConfidencePerFrame[frameNumber]
            .Select(x => x with { Z = 0 }).ToList();
        List<Vector3> flattenedFollow = recenteredFollowPoseAndConfidencePerFrame[frameNumber]
            .Select(x => x with { Z = 0 }).ToList();

        List<Vector3> leadProjectionsAtThisFrame = Adjusted(flattenedLead, frameNumber);
        LeadProjectionsPerFrame[frameNumber] = leadProjectionsAtThisFrame;

        List<Vector3> followProjectionsAtThisFrame = Adjusted(flattenedFollow, frameNumber);
        FollowProjectionsPerFrame[frameNumber] = followProjectionsAtThisFrame;
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
                     recenteredLeadPoseAndConfidencePerFrame[frameNumber][i].Z; // confidence
        }

        for (int i = 0; i < merged3DPoseFollow.Count; i++)
        {
            Vector3 vector3 = merged3DPoseFollow[i];
            Vector3 target = Vector3.Normalize(vector3 - PositionsPerFrame[frameNumber]);
            Vector3 keypoint = Vector3.Normalize(
                FollowProjectionsPerFrame[frameNumber][i] - PositionsPerFrame[frameNumber]);

            // find the angle between the vectors
            error += MathF.Acos(Vector3.Dot(target, keypoint)) *
                     recenteredFollowPoseAndConfidencePerFrame[frameNumber][i].Z; // confidence
        }

        return error;
    }

    public void AddPosesAndRecenterAndScaleToCamera(
        List<List<Vector3>> lead2DPixelsAndConfidence,
        List<List<Vector3>> follow2PixelsAndConfidence,
        List<List<Vector3>> mirrorLead2DPixelsAndConfidence,
        List<List<Vector3>> mirrorFollow2DPixelsAndConfidence)
    {
        leadPoseAndConfidencePerFrame = lead2DPixelsAndConfidence;
        followPoseAndConfidencePerFrame = follow2PixelsAndConfidence;
        mirrorLeadPoseAndConfidencePerFrame = mirrorLead2DPixelsAndConfidence;
        mirrorFollowPoseAndConfidencePerFrame = mirrorFollow2DPixelsAndConfidence;
        
        recenteredLeadPoseAndConfidencePerFrame = lead2DPixelsAndConfidence.Select(listVec => listVec.Select(vec => new Vector3(
                (vec.X - Size.X / 2) * PixelToMeter,
                -(vec.Y - Size.Y / 2) * PixelToMeter, // flip
                vec.Z)) // keep the confidence
            .ToList()).ToList();

        recenteredFollowPoseAndConfidencePerFrame = follow2PixelsAndConfidence.Select(listVec => listVec.Select(vec => new Vector3(
                (vec.X - Size.X / 2) * PixelToMeter,
                -(vec.Y - Size.Y / 2) * PixelToMeter, // flip
                vec.Z)) // keep the confidence
            .ToList()).ToList();

        FollowProjectionsPerFrame.Add([]);
        LeadProjectionsPerFrame.Add([]);
    }
    
    public List<List<Vector3>> PosesPerDancerAtFrame(int frameNumber)
    {
        return
        [
            leadPoseAndConfidencePerFrame[frameNumber],
            followPoseAndConfidencePerFrame[frameNumber],
            mirrorLeadPoseAndConfidencePerFrame[frameNumber],
            mirrorFollowPoseAndConfidencePerFrame[frameNumber]
        ];
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

    public bool HasPoseAtFrame(int frameNumber, bool isLead)
    {
        return isLead
            ? recenteredLeadPoseAndConfidencePerFrame[frameNumber].Count > 0
            : recenteredFollowPoseAndConfidencePerFrame[frameNumber].Count > 0;
    }

    public Ray PoseRay(int frameNumber, int jointNumber, bool isLead)
    {
        Ray rayToJoint = new Ray(
            PositionsPerFrame[frameNumber],
            Vector3.Normalize(isLead
                ? LeadProjectionsPerFrame[frameNumber][jointNumber] - PositionsPerFrame[frameNumber]
                : FollowProjectionsPerFrame[frameNumber][jointNumber] - PositionsPerFrame[frameNumber]));
        return rayToJoint;
    }

    public float JointConfidence(int frameNumber, int jointNumber, bool isLead)
    {
        return isLead
            ? recenteredLeadPoseAndConfidencePerFrame[frameNumber][jointNumber].Z
            : recenteredFollowPoseAndConfidencePerFrame[frameNumber][jointNumber].Z;
    }
    
    [Serializable] 
    public class PositionAndRotation 
    { 
        public float positionX; 
        public float positionY; 
        public float positionZ; 
 
        public float rotationX; 
        public float rotationY; 
        public float rotationZ; 
        public float rotationW; 
     
        public float focal; 
    } 
}