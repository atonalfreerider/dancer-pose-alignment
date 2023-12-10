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
        Vector3.UnitZ,
        RotationsPerFrame[frame]);

    public Vector3 Up(int frame) => Vector3.Transform(
        Vector3.UnitY,
        RotationsPerFrame[frame]);

    public Vector3 Right(int frame) => Vector3.Transform(
        Vector3.UnitX,
        RotationsPerFrame[frame]);

    readonly List<List<Vector3>> leadProjectionsPerFrame = [];
    readonly List<List<Vector3>> followProjectionsPerFrame = [];

    List<List<Vector3>> leadPoseAndConfidencePerFrame;
    List<List<Vector3>> followPoseAndConfidencePerFrame;
    List<List<Vector3>> mirrorLeadPoseAndConfidencePerFrame;
    List<List<Vector3>> mirrorFollowPoseAndConfidencePerFrame;
    List<List<Vector3>> otherCamerasPositionAndConfidencePerFrame;
    List<List<Vector3>> mirrorOtherCamerasRotationAndConfidencePerFrame;

    List<List<Vector3>> recenteredLeadPoseAndConfidencePerFrame;
    List<List<Vector3>> recenteredFollowPoseAndConfidencePerFrame;
    List<List<Vector3>> mirrorRecenteredLeadPoseAndConfidencePerFrame;
    List<List<Vector3>> mirrorRecenteredFollowPoseAndConfidencePerFrame;
    List<List<Vector3>> recenteredOtherCamerasPositionAndConfidencePerFrame;
    List<List<Vector3>> mirrorRecenteredOtherCamerasRotationAndConfidencePerFrame;

    public void Project(int frameNumber)
    {
        List<Vector3> flattenedLead = recenteredLeadPoseAndConfidencePerFrame[frameNumber]
            .Select(x => x with { Z = 0 }).ToList();
        List<Vector3> flattenedFollow = recenteredFollowPoseAndConfidencePerFrame[frameNumber]
            .Select(x => x with { Z = 0 }).ToList();

        List<Vector3> leadProjectionsAtThisFrame = Adjusted(flattenedLead, frameNumber);
        leadProjectionsPerFrame[frameNumber] = leadProjectionsAtThisFrame;

        List<Vector3> followProjectionsAtThisFrame = Adjusted(flattenedFollow, frameNumber);
        followProjectionsPerFrame[frameNumber] = followProjectionsAtThisFrame;
    }

    public Vector2 ReverseProjectPoint(Vector3 worldPoint, int frameNumber)
    {
        Vector3 target = TargetAtFrame(worldPoint, frameNumber);
        Vector2 imagePlaneCoordinates = GetImagePlaneCoordinates(target, frameNumber);

        Vector2 offcenterAndRescaleAndFlip = new Vector2(
            imagePlaneCoordinates.X / PixelToMeter + Size.X / 2,
            -imagePlaneCoordinates.Y / PixelToMeter + Size.Y / 2);

        // don't render off screen
        if (offcenterAndRescaleAndFlip.Y < 0)
        {
            offcenterAndRescaleAndFlip.Y = 0;
        }

        if (offcenterAndRescaleAndFlip.Y > Size.Y)
        {
            offcenterAndRescaleAndFlip.Y = Size.Y;
        }

        if (offcenterAndRescaleAndFlip.X < 0)
        {
            offcenterAndRescaleAndFlip.X = 0;
        }

        if (offcenterAndRescaleAndFlip.X > Size.X)
        {
            offcenterAndRescaleAndFlip.X = Size.X;
        }

        return offcenterAndRescaleAndFlip;
    }

    Vector2 GetImagePlaneCoordinates(Vector3 rayDirection, int frameNumber)
    {
        // Calculate the intersection point with the image plane
        float t = FocalLength / Vector3.Dot(Forward(frameNumber), rayDirection);
        Vector3 intersectionPoint = t * rayDirection;

        // Calculate the coordinates relative to the image plane center
        Vector2 imagePlaneCoordinates = new Vector2(
            Vector3.Dot(intersectionPoint, Right(frameNumber)),
            Vector3.Dot(intersectionPoint, Up(frameNumber)));

        return imagePlaneCoordinates;
    }

    public float Error(List<Vector3> merged3DPoseLead, List<Vector3> merged3DPoseFollow, int frameNumber)
    {
        float error = 0;

        for (int i = 0; i < merged3DPoseLead.Count; i++)
        {
            if (leadProjectionsPerFrame[frameNumber].Count <= i) continue;
            Vector3 target = TargetAtFrame(merged3DPoseLead[i], frameNumber);
            Vector3 keypoint = TargetAtFrame(leadProjectionsPerFrame[frameNumber][i], frameNumber);

            // find the angle between the vectors
            error += MathF.Acos(Vector3.Dot(target, keypoint)) *
                     recenteredLeadPoseAndConfidencePerFrame[frameNumber][i].Z; // confidence
        }

        for (int i = 0; i < merged3DPoseFollow.Count; i++)
        {
            if (followProjectionsPerFrame[frameNumber].Count <= i) continue;
            Vector3 target = TargetAtFrame(merged3DPoseFollow[i], frameNumber);
            Vector3 keypoint = TargetAtFrame(followProjectionsPerFrame[frameNumber][i], frameNumber);

            // find the angle between the vectors
            error += MathF.Acos(Vector3.Dot(target, keypoint)) *
                     recenteredFollowPoseAndConfidencePerFrame[frameNumber][i].Z; // confidence
        }

        return error;
    }

    /// <summary>
    /// Provides a normalized vector from this camera to a target in 3D space at a given frame
    /// </summary>
    Vector3 TargetAtFrame(Vector3 vector3, int frameNumber)
    {
        return Vector3.Normalize(vector3 - PositionsPerFrame[frameNumber]);
    }

    public void AddPosesAndRecenterAndScaleToCamera(
        List<List<Vector3>> lead2DPixelsAndConfidence,
        List<List<Vector3>> follow2PixelsAndConfidence,
        List<List<Vector3>> mirrorLead2DPixelsAndConfidence,
        List<List<Vector3>> mirrorFollow2DPixelsAndConfidence,
        List<List<Vector3>> otherCamerasPositionAndConfidence,
        List<List<Vector3>> mirrorOtherCamerasRotationAndConfidence)
    {
        leadPoseAndConfidencePerFrame = lead2DPixelsAndConfidence;
        followPoseAndConfidencePerFrame = follow2PixelsAndConfidence;
        mirrorLeadPoseAndConfidencePerFrame = mirrorLead2DPixelsAndConfidence;
        mirrorFollowPoseAndConfidencePerFrame = mirrorFollow2DPixelsAndConfidence;
        otherCamerasPositionAndConfidencePerFrame = otherCamerasPositionAndConfidence;
        mirrorOtherCamerasRotationAndConfidencePerFrame = mirrorOtherCamerasRotationAndConfidence;

        recenteredLeadPoseAndConfidencePerFrame = lead2DPixelsAndConfidence.Select(listVec => listVec.Select(vec =>
                new Vector3(
                    (vec.X - Size.X / 2) * PixelToMeter,
                    -(vec.Y - Size.Y / 2) * PixelToMeter, // flip
                    vec.Z)) // keep the confidence
            .ToList()).ToList();

        recenteredFollowPoseAndConfidencePerFrame = follow2PixelsAndConfidence.Select(listVec => listVec.Select(vec =>
                new Vector3(
                    (vec.X - Size.X / 2) * PixelToMeter,
                    -(vec.Y - Size.Y / 2) * PixelToMeter, // flip
                    vec.Z)) // keep the confidence
            .ToList()).ToList();

        recenteredOtherCamerasPositionAndConfidencePerFrame = otherCamerasPositionAndConfidencePerFrame.Select(
            listVec => listVec.Select(vec =>
                    new Vector3(
                        (vec.X - Size.X / 2) * PixelToMeter,
                        -(vec.Y - Size.Y / 2) * PixelToMeter, // flip
                        vec.Z)) // keep the confidence
                .ToList()).ToList();

        mirrorRecenteredOtherCamerasRotationAndConfidencePerFrame = mirrorOtherCamerasRotationAndConfidencePerFrame
            .Select(listVec => listVec.Select(vec =>
                    new Vector3(
                        (vec.X - Size.X / 2) * PixelToMeter,
                        -(vec.Y - Size.Y / 2) * PixelToMeter, // flip
                        vec.Z)) // keep the confidence
                .ToList()).ToList();


        for (int i = 0; i < lead2DPixelsAndConfidence.Count; i++)
        {
            leadProjectionsPerFrame.Add([]);
        }

        for (int i = 0; i < follow2PixelsAndConfidence.Count; i++)
        {
            followProjectionsPerFrame.Add([]);
        }
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

    public List<Vector3> OtherCamerasPositionAndConfidenceAtFrame(int frameNumber)
    {
        return recenteredOtherCamerasPositionAndConfidencePerFrame[frameNumber];
    }

    public List<Vector3> OtherMirrorCamerasPositionAndConfidenceAtFrame(int frameNumber)
    {
        return mirrorRecenteredOtherCamerasRotationAndConfidencePerFrame[frameNumber];
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

    public void Home()
    {
        Vector2 leadLeftAnkle = new Vector2(
            leadPoseAndConfidencePerFrame[0][(int)Halpe.LAnkle].X,
            leadPoseAndConfidencePerFrame[0][(int)Halpe.LAnkle].Y);

        Vector2 leadHip = new Vector2(
            leadPoseAndConfidencePerFrame[0][(int)Halpe.LHip].X,
            leadPoseAndConfidencePerFrame[0][(int)Halpe.LHip].Y);

        const float hipHeight = .75f;

        for (int j = 0; j < 4; j++)
        {
            Vector2 origin = ReverseProjectPoint(Vector3.Zero, 0);
            while (leadLeftAnkle.X < origin.X)
            {
                RotationsPerFrame[0] *= Quaternion.CreateFromAxisAngle(Vector3.UnitY, .001f);
                origin = ReverseProjectPoint(Vector3.Zero, 0);
            }

            while (leadLeftAnkle.X > origin.X)
            {
                RotationsPerFrame[0] *= Quaternion.CreateFromAxisAngle(Vector3.UnitY, -.001f);
                origin = ReverseProjectPoint(Vector3.Zero, 0);
            }

            Vector2 worldHip = ReverseProjectPoint(new Vector3(0, hipHeight, 0), 0);
            for (int i = 0; i < 500; i++)
            {
                if (leadLeftAnkle.Y > origin.Y && leadHip.Y > worldHip.Y)
                {
                    // pitch up
                    RotationsPerFrame[0] *= Quaternion.CreateFromAxisAngle(Vector3.UnitX, -.001f);
                }
                else if (leadLeftAnkle.Y < origin.Y && leadHip.Y < worldHip.Y)
                {
                    // pitch down
                    RotationsPerFrame[0] *= Quaternion.CreateFromAxisAngle(Vector3.UnitX, .001f);
                }
                else if (leadLeftAnkle.Y > origin.Y && leadHip.Y < worldHip.Y)
                {
                    FocalLength += .001f;
                }
                else if (leadLeftAnkle.Y < origin.Y && leadHip.Y > worldHip.Y)
                {
                    FocalLength -= .001f;
                }

                origin = ReverseProjectPoint(Vector3.Zero, 0);
                worldHip = ReverseProjectPoint(new Vector3(0, hipHeight, 0), 0);
            }
        }
    }

    public bool TestSixTranslations(List<Vector3> merged3DPoseLeadPerFrame, List<Vector3> merged3DPoseFollowPerFrame,
        int frame)
    {
        float currentError = Error(
            merged3DPoseLeadPerFrame,
            merged3DPoseFollowPerFrame,
            frame);

        Vector3 currentCameraPosition = PositionsPerFrame[frame];
        Quaternion currentCameraRotation = RotationsPerFrame[frame];

        // translate left
        PositionsPerFrame[frame] += Right(frame) * .05f;

        if (frame == 0)
        {
            Home();
        }

        Vector3 leftCameraPosition = PositionsPerFrame[frame];
        Quaternion leftCameraRotation = RotationsPerFrame[frame];
        float leftError = Error(
            merged3DPoseLeadPerFrame,
            merged3DPoseFollowPerFrame,
            frame);

        PositionsPerFrame[frame] = currentCameraPosition;
        RotationsPerFrame[frame] = currentCameraRotation;

        // translate right
        PositionsPerFrame[frame] -= Right(frame) * .05f;
        if (frame == 0)
        {
            Home();
        }

        Vector3 rightCameraPosition = PositionsPerFrame[frame];
        Quaternion rightCameraRotation = RotationsPerFrame[frame];

        float rightError = Error(
            merged3DPoseLeadPerFrame,
            merged3DPoseFollowPerFrame,
            frame);

        PositionsPerFrame[frame] = currentCameraPosition;
        RotationsPerFrame[frame] = currentCameraRotation;

        // translate up
        PositionsPerFrame[frame] += Up(frame) * .05f;

        if (frame == 0)
        {
            Home();
        }

        Vector3 upCameraPosition = PositionsPerFrame[frame];
        Quaternion upCameraRotation = RotationsPerFrame[frame];

        float upError = Error(
            merged3DPoseLeadPerFrame,
            merged3DPoseFollowPerFrame,
            frame);

        // translate down
        PositionsPerFrame[frame] -= Up(frame) * .05f;
        if (frame == 0)
        {
            Home();
        }
        
        Vector3 downCameraPosition = PositionsPerFrame[frame];
        Quaternion downCameraRotation = RotationsPerFrame[frame];
        
        float downError = Error(
            merged3DPoseLeadPerFrame,
            merged3DPoseFollowPerFrame,
            frame);
        
        PositionsPerFrame[frame] = currentCameraPosition;
        RotationsPerFrame[frame] = currentCameraRotation;
        
        // translate forward
        PositionsPerFrame[frame] += Forward(frame) * .05f;
        if (frame == 0)
        {
            Home();
        }
        
        Vector3 forwardCameraPosition = PositionsPerFrame[frame];
        Quaternion forwardCameraRotation = RotationsPerFrame[frame];
        
        float forwardError = Error(
            merged3DPoseLeadPerFrame,
            merged3DPoseFollowPerFrame,
            frame);
        
        PositionsPerFrame[frame] = currentCameraPosition;
        RotationsPerFrame[frame] = currentCameraRotation;
        
        // translate backward
        PositionsPerFrame[frame] -= Forward(frame) * .05f;
        if (frame == 0)
        {
            Home();
        }
        
        Vector3 backwardCameraPosition = PositionsPerFrame[frame];
        Quaternion backwardCameraRotation = RotationsPerFrame[frame];
        
        float backwardError = Error(
            merged3DPoseLeadPerFrame,
            merged3DPoseFollowPerFrame,
            frame);
        
        if(leftError < rightError && leftError < upError && leftError < downError && leftError < forwardError && leftError < backwardError)
        {
            PositionsPerFrame[frame] = leftCameraPosition;
            RotationsPerFrame[frame] = leftCameraRotation;
            return true;
        }
        if(rightError < leftError && rightError < upError && rightError < downError && rightError < forwardError && rightError < backwardError)
        {
            PositionsPerFrame[frame] = rightCameraPosition;
            RotationsPerFrame[frame] = rightCameraRotation;
            return true;
        }
        if(upError < leftError && upError < rightError && upError < downError && upError < forwardError && upError < backwardError)
        {
            PositionsPerFrame[frame] = upCameraPosition;
            RotationsPerFrame[frame] = upCameraRotation;
            return true;
        }
        if(downError < leftError && downError < rightError && downError < upError && downError < forwardError && downError < backwardError)
        {
            PositionsPerFrame[frame] = downCameraPosition;
            RotationsPerFrame[frame] = downCameraRotation;
            return true;
        }
        if(forwardError < leftError && forwardError < rightError && forwardError < upError && forwardError < downError && forwardError < backwardError)
        {
            PositionsPerFrame[frame] = forwardCameraPosition;
            RotationsPerFrame[frame] = forwardCameraRotation;
            return true;
        }
        if(backwardError < leftError && backwardError < rightError && backwardError < upError && backwardError < downError && backwardError < forwardError)
        {
            PositionsPerFrame[frame] = backwardCameraPosition;
            RotationsPerFrame[frame] = backwardCameraRotation;
            return true;
        }

        PositionsPerFrame[frame] = currentCameraPosition;
        RotationsPerFrame[frame] = currentCameraRotation;
        return false;

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
                ? leadProjectionsPerFrame[frameNumber][jointNumber] - PositionsPerFrame[frameNumber]
                : followProjectionsPerFrame[frameNumber][jointNumber] - PositionsPerFrame[frameNumber]));
        return rayToJoint;
    }

    public float JointConfidence(int frameNumber, int jointNumber, bool isLead)
    {
        return isLead
            ? recenteredLeadPoseAndConfidencePerFrame[frameNumber][jointNumber].Z
            : recenteredFollowPoseAndConfidencePerFrame[frameNumber][jointNumber].Z;
    }

    public void CopyPositionsToNextFrame(int frameNumber)
    {
        if (PositionsPerFrame.Count <= frameNumber + 1)
        {
            PositionsPerFrame.Add(PositionsPerFrame[frameNumber - 1]);
        }

        if (RotationsPerFrame.Count <= frameNumber + 1)
        {
            RotationsPerFrame.Add(RotationsPerFrame[frameNumber - 1]);
        }
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