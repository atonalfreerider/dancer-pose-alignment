﻿using System.Numerics;

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

    readonly List<List<Vector3>> leadProjectionsPerFrame = [];
    readonly List<List<Vector3>> followProjectionsPerFrame = [];

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
            Vector3 target = TargetAtFrame(merged3DPoseLead[i], frameNumber);
            Vector3 keypoint = TargetAtFrame(leadProjectionsPerFrame[frameNumber][i], frameNumber);

            // find the angle between the vectors
            error += MathF.Acos(Vector3.Dot(target, keypoint)) *
                     recenteredLeadPoseAndConfidencePerFrame[frameNumber][i].Z; // confidence
        }

        for (int i = 0; i < merged3DPoseFollow.Count; i++)
        {
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
        List<List<Vector3>> mirrorFollow2DPixelsAndConfidence)
    {
        leadPoseAndConfidencePerFrame = lead2DPixelsAndConfidence;
        followPoseAndConfidencePerFrame = follow2PixelsAndConfidence;
        mirrorLeadPoseAndConfidencePerFrame = mirrorLead2DPixelsAndConfidence;
        mirrorFollowPoseAndConfidencePerFrame = mirrorFollow2DPixelsAndConfidence;

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

        followProjectionsPerFrame.Add([]);
        leadProjectionsPerFrame.Add([]);
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