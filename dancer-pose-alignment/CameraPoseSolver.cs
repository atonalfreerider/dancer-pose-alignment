using System.Numerics;
using System.Reflection;
using ComputeSharp;
using Newtonsoft.Json;

namespace dancer_pose_alignment;

public class CameraPoseSolver(PoseType poseType)
{
    // these are the 3D pose values that will be output
    readonly List<List<Vector3>> merged3DPoseLeadPerFrame = [];
    readonly List<List<Vector3>> merged3DPoseFollowPerFrame = [];

    readonly Dictionary<string, CameraSetup> cameras = [];
    int frameNumber = 0;
    public int MaximumFrameCount = int.MaxValue;

    // drawing variables
    Dictionary<string, Vector3> cameraPositions => cameras.ToDictionary(
        pair => pair.Key,
        pair => pair.Value.Position);

    readonly List<Vector3> originCross =
    [
        Vector3.Zero,
        Vector3.UnitX,
        Vector3.UnitY,
        Vector3.UnitZ
    ];

    public void CreateCamera(
        string name,
        Vector2 imageSize,
        int frameCount,
        int startingFrame,
        int maxFrame,
        string dbPath)
    {
        CameraSetup camera = new(name, imageSize, frameCount, poseType, startingFrame, maxFrame, dbPath);
        cameras.Add(name, camera);
        camera.SetAllPosesAtFrame(0);
    }

    public void SetAllAffine(List<Vector3> affine, string camName)
    {
        cameras[camName].SetAllAffine(affine);
    }

    public bool Advance()
    {
        if (frameNumber >= MaximumFrameCount - 1) return false;

        frameNumber++;
        Console.WriteLine(frameNumber);
        foreach ((string videoFilePath, CameraSetup cameraSetup) in cameras)
        {
            cameraSetup.CopyRotationToNextFrame(frameNumber);
            cameras[videoFilePath].SetAllPosesAtFrame(frameNumber);
            cameraSetup.UpdateKalman(frameNumber);
        }

        return true;
    }

    public void SetFrame(int frame)
    {
        frameNumber = frame;
    }

    public bool Rewind()
    {
        if (frameNumber <= 0) return false;

        frameNumber--;
        return true;
    }

    public void CalculateLeadFollow3DPoses()
    {
        foreach (CameraSetup cameraSetup in cameras.Values)
        {
            cameraSetup.Project(frameNumber);
        }

        while (merged3DPoseLeadPerFrame.Count <= frameNumber)
        {
            // add a new frame
            merged3DPoseLeadPerFrame.Add([]);
        }

        while (merged3DPoseFollowPerFrame.Count <= frameNumber)
        {
            merged3DPoseFollowPerFrame.Add([]);
        }

        merged3DPoseLeadPerFrame[frameNumber].Clear();
        merged3DPoseFollowPerFrame[frameNumber].Clear();

        List<Vector3> merged3DPoseLead = Calculate3DPose(true);
        List<Vector3> merged3DPoseFollow = Calculate3DPose(false);

        merged3DPoseLeadPerFrame[frameNumber] = merged3DPoseLead;
        merged3DPoseFollowPerFrame[frameNumber] = merged3DPoseFollow;
    }

    List<Vector3> Calculate3DPose(bool isLead)
    {
        List<float3> rayOriginPerCameraPerJoint = []; // in batches of camera count
        List<float3> rayDirectionPerCameraPerJoint = []; // in batches of camera count
        List<float> jointConfidencePerCameraPerJoint = []; // in batches of camera count

        int arrayLength = JointExtension.PoseCount(poseType) * cameras.Count;

        for (int i = 0; i < JointExtension.PoseCount(poseType); i++)
        {
            foreach (CameraSetup cameraSetup in cameras.Values)
            {
                if (cameraSetup.HasPoseAtFrame(frameNumber, isLead))
                {
                    Ray ray = cameraSetup.PoseRay(frameNumber, i, isLead);
                    rayOriginPerCameraPerJoint.Add(ray.Origin);
                    rayDirectionPerCameraPerJoint.Add(ray.Direction);
                    jointConfidencePerCameraPerJoint.Add(cameraSetup.JointConfidence(frameNumber, i, isLead));
                }
                else
                {
                    rayOriginPerCameraPerJoint.Add(float3.Zero);
                    rayDirectionPerCameraPerJoint.Add(float3.UnitZ);
                    jointConfidencePerCameraPerJoint.Add(0);
                }
            }
        }

        float3[] jointMidpoints = new float3[JointExtension.PoseCount(poseType)];

        using ReadWriteBuffer<float3> minMidpointBuffer =
            GraphicsDevice.GetDefault().AllocateReadWriteBuffer(jointMidpoints);
        using ReadOnlyBuffer<float3> rayOriginBuffer =
            GraphicsDevice.GetDefault().AllocateReadOnlyBuffer(rayOriginPerCameraPerJoint.ToArray());
        using ReadOnlyBuffer<float3> rayDirectionBuffer = GraphicsDevice.GetDefault()
            .AllocateReadOnlyBuffer(rayDirectionPerCameraPerJoint.ToArray());
        using ReadOnlyBuffer<float> jointConfidenceBuffer = GraphicsDevice.GetDefault()
            .AllocateReadOnlyBuffer(jointConfidencePerCameraPerJoint.ToArray());

        MidpointFinder midpointFinder = new(
            minMidpointBuffer,
            rayOriginBuffer,
            rayDirectionBuffer,
            jointConfidenceBuffer,
            cameras.Count);

        try
        {
            GraphicsDevice.GetDefault().For(arrayLength, midpointFinder);
        }
        catch (TargetInvocationException e)
        {
            Console.WriteLine(e);
        }

        List<Vector3> scatterPose = minMidpointBuffer.ToArray()
            .Select(result => new Vector3(result.X, result.Y, result.Z))
            .ToList();

        return scatterPose;
    }

    /// <summary>
    /// To be used as the final anchored to the floor, consistent limb length model
    /// </summary>
    /// <returns></returns>
    static List<Vector3> AnchorFinalPose(
        IReadOnlyList<Vector3> scatterPose, 
        PoseType poseType,
        float calfLength,
        float thighLength,
        float upperArmLength,
        float forearmLength)
    {
        List<Vector3> final3DPose = [];
        for (int i = 0; i < JointExtension.PoseCount(poseType); i++)
        {
            final3DPose.Add(Vector3.Zero);
        }

        // ANCHOR LEGS
        Vector3 rAnklePos = scatterPose[JointExtension.RAnkleIndex(poseType)];
        Vector3 lAnklePos = scatterPose[JointExtension.LAnkleIndex(poseType)];

        Vector3 rKneePos = scatterPose[JointExtension.RKneeIndex(poseType)];
        Vector3 lKneePos = scatterPose[JointExtension.LKneeIndex(poseType)];

        Vector3 rHipPos = scatterPose[JointExtension.RHipIndex(poseType)];
        Vector3 lHipPos = scatterPose[JointExtension.LHipIndex(poseType)];

        if (rAnklePos.Y < lAnklePos.Y)
        {
            // right ankle is lower to the ground
            final3DPose[JointExtension.RAnkleIndex(poseType)] = rAnklePos with { Y = 0 };
            rKneePos = rAnklePos + Vector3.Normalize(rKneePos - rAnklePos) * calfLength;
            final3DPose[JointExtension.RKneeIndex(poseType)] = rKneePos;
            rHipPos = rKneePos + Vector3.Normalize(rHipPos - rKneePos) * thighLength;
            final3DPose[JointExtension.RHipIndex(poseType)] = rHipPos;

            final3DPose[JointExtension.LHipIndex(poseType)] = lHipPos;
            lKneePos = lHipPos + Vector3.Normalize(lKneePos - lHipPos) * thighLength;
            final3DPose[JointExtension.LKneeIndex(poseType)] = lKneePos;
            lAnklePos = lKneePos + Vector3.Normalize(lAnklePos - lKneePos) * calfLength;
            final3DPose[JointExtension.LAnkleIndex(poseType)] = lAnklePos;
        }
        else
        {
            final3DPose[JointExtension.LAnkleIndex(poseType)] = lAnklePos;
            lKneePos = lAnklePos + Vector3.Normalize(lKneePos - lAnklePos) * calfLength;
            final3DPose[JointExtension.LKneeIndex(poseType)] = lKneePos;
            lHipPos = lKneePos + Vector3.Normalize(lHipPos - lKneePos) * thighLength;
            final3DPose[JointExtension.LHipIndex(poseType)] = lHipPos;

            final3DPose[JointExtension.RHipIndex(poseType)] = rHipPos;
            rKneePos = rHipPos + Vector3.Normalize(rKneePos - rHipPos) * thighLength;
            final3DPose[JointExtension.RKneeIndex(poseType)] = rKneePos;
            rAnklePos = rKneePos + Vector3.Normalize(rAnklePos - rKneePos) * calfLength;
            final3DPose[JointExtension.RAnkleIndex(poseType)] = rAnklePos;
        }

        // ANCHOR ARMS
        Vector3 rShoulderPos = scatterPose[JointExtension.RShoulderIndex(poseType)];
        Vector3 lShoulderPos = scatterPose[JointExtension.LShoulderIndex(poseType)];

        Vector3 rElbowPos = scatterPose[JointExtension.RElbowIndex(poseType)];
        Vector3 lElbowPos = scatterPose[JointExtension.LElbowIndex(poseType)];

        Vector3 rWristPos = scatterPose[JointExtension.RWristIndex(poseType)];
        Vector3 lWristPos = scatterPose[JointExtension.LWristIndex(poseType)];

        final3DPose[JointExtension.RShoulderIndex(poseType)] = rShoulderPos;
        rElbowPos = rShoulderPos + Vector3.Normalize(rElbowPos - rShoulderPos) * upperArmLength;
        final3DPose[JointExtension.RElbowIndex(poseType)] = rElbowPos;
        rWristPos = rElbowPos + Vector3.Normalize(rWristPos - rElbowPos) * forearmLength;
        final3DPose[JointExtension.RWristIndex(poseType)] = rWristPos;

        final3DPose[JointExtension.LShoulderIndex(poseType)] = lShoulderPos;
        lElbowPos = lShoulderPos + Vector3.Normalize(lElbowPos - lShoulderPos) * upperArmLength;
        final3DPose[JointExtension.LElbowIndex(poseType)] = lElbowPos;
        lWristPos = lElbowPos + Vector3.Normalize(lWristPos - lElbowPos) * forearmLength;
        final3DPose[JointExtension.LWristIndex(poseType)] = lWristPos;

        final3DPose[JointExtension.NoseIndex(poseType)] = scatterPose[JointExtension.NoseIndex(poseType)];
        final3DPose[JointExtension.LEarIndex(poseType)] = scatterPose[JointExtension.LEarIndex(poseType)];
        final3DPose[JointExtension.REarIndex(poseType)] = scatterPose[JointExtension.REarIndex(poseType)];
        final3DPose[JointExtension.LEyeIndex(poseType)] = scatterPose[JointExtension.LEyeIndex(poseType)];
        final3DPose[JointExtension.REyeIndex(poseType)] = scatterPose[JointExtension.REyeIndex(poseType)];

        return final3DPose;
    }

    #region USER MARKUP

    /// <summary>
    /// For the selected camera, attempt to select the figure from the closes joint
    /// </summary>
    /// <returns>The index of the dancer and the index of the joint</returns>
    public Tuple<PoseBoundingBox, int> MarkDancerAtCam(string camName, Vector2 click, string selectedButton)
    {
        return cameras[camName].MarkDancer(click, frameNumber, selectedButton);
    }

    public void MoveKeypointAtCam(string camName, Vector2 click, Tuple<PoseBoundingBox?, int> selectedPoseAndKeypoint)
    {
        cameras[camName].MoveKeypoint(click, frameNumber, selectedPoseAndKeypoint);
    }

    #endregion

    #region ITERATORS

    public void HomeAllCameras()
    {
        if (frameNumber > 0) return;
        foreach (CameraSetup cam in cameras.Values)
        {
            cam.Home();
        }
    }

    public void SetCamR()
    {
        if (frameNumber > 0) return;

        List<Tuple<float, float>> cameraWall = OrderedAndSmoothedCameraWall();
        foreach (CameraSetup camerasValue in cameras.Values)
        {
            camerasValue.SetRadiusFromCameraWall(cameraWall);
        }
    }

    #endregion

    #region DRAWING

    public List<PoseBoundingBox> GetPosesAtFrameAtCamera(string camName)
    {
        return cameras[camName].GetPosesPerDancerAtFrame(frameNumber);
    }

    public List<Vector2> ReverseProjectionOfPoseAtCamera(string camName, bool isLead)
    {
        if (isLead)
        {
            if (merged3DPoseLeadPerFrame.Count <= frameNumber) return [];

            List<Vector2> reverseLeadProjection = merged3DPoseLeadPerFrame[frameNumber]
                .Select(vec => cameras[camName].ReverseProjectPoint(vec, frameNumber)).ToList();

            return reverseLeadProjection;
        }

        if (merged3DPoseFollowPerFrame.Count <= frameNumber) return [];

        List<Vector2> reverseFollowProjection = merged3DPoseFollowPerFrame[frameNumber]
            .Select(vec => cameras[camName].ReverseProjectPoint(vec, frameNumber)).ToList();

        return reverseFollowProjection;
    }

    public List<Vector2> ReverseProjectOriginCrossAtCamera(string camName)
    {
        return originCross.Select(vec => cameras[camName]
            .ReverseProjectPoint(vec, frameNumber)).ToList();
    }

    public List<Vector2> ReverseProjectCameraPositionsAtCameraAndManualPair(string camName)
    {
        List<Vector2> pointPairs = [];
        foreach ((string otherCamName, Vector3 camPos) in cameraPositions)
        {
            if (camName == otherCamName) continue;

            Vector2 point = cameras[camName].ReverseProjectPoint(camPos, frameNumber);
            pointPairs.Add(point);
        }

        return pointPairs;
    }

    #endregion

    #region REFERENCE

    List<Tuple<float, float>> OrderedAndSmoothedCameraWall()
    {
        List<Tuple<float, float>> ordered = cameras.Values
            .SelectMany(cam => cam.CameraWall).OrderBy(x => x.Item1).ToList();
        List<Vector3> allPoints = [];
        foreach (Tuple<float, float> tuple in ordered)
        {
            Vector3 point = new(tuple.Item2 * MathF.Sin(tuple.Item1), 0, tuple.Item2 * MathF.Cos(tuple.Item1));
            allPoints.Add(point);
        }

        allPoints = Transform.MovingAverageSmoothing(allPoints, 4);

        List<Tuple<float, float>> final = [];
        for (int i = 0; i < allPoints.Count; i++)
        {
            Vector3 point = allPoints[i];
            float rad = Vector3.Distance(Vector3.Zero, point);
            final.Add(new Tuple<float, float>(ordered[i].Item1, rad));
        }

        return final;
    }

    public int CurrentFrame => frameNumber;

    static float MedianBoneLength(IEnumerable<Vector3> jointSet1, IReadOnlyList<Vector3> jointSet2)
    {
        List<float> boneLengths = jointSet1
            .Select((vec1, i) => Vector3.Distance(vec1, jointSet2[i])).ToList();

        boneLengths.Sort();
        return boneLengths[boneLengths.Count / 2];
    }

    #endregion

    public void ClearAfterFrame(int frameToClearAfter)
    {
        foreach (CameraSetup cameraSetup in cameras.Values)
        {
            cameraSetup.ClearAfterFrame(frameToClearAfter);
        }
    }

    List<List<Vector3>> AnchorAndSmooth(IReadOnlyCollection<List<Vector3>> list)
    {
        // body limb constants so that 3D pose is constrained
        List<Vector3> allRightAnkles = list.Select(pose => pose[JointExtension.RAnkleIndex(poseType)]).ToList();
        List<Vector3> allRightKnees = list.Select(pose => pose[JointExtension.RKneeIndex(poseType)]).ToList();
        List<Vector3> allRightHips = list.Select(pose => pose[JointExtension.RHipIndex(poseType)]).ToList();
        
        float calfLength = MedianBoneLength(allRightAnkles, allRightKnees);
        float thighLength = MedianBoneLength(allRightKnees, allRightHips);
        
        List<Vector3> allLeftElbows = list.Select(pose => pose[JointExtension.LElbowIndex(poseType)]).ToList();
        List<Vector3> allLeftShoulders = list.Select(pose => pose[JointExtension.LShoulderIndex(poseType)]).ToList();
        List<Vector3> allLeftWrists = list.Select(pose => pose[JointExtension.LWristIndex(poseType)]).ToList();
        
        float upperArmLengthLeft = MedianBoneLength(allLeftShoulders, allLeftElbows);
        float forearmLengthLeft = MedianBoneLength(allLeftElbows, allLeftWrists);
        
        List<Vector3> allRightElbows = list.Select(pose => pose[JointExtension.RElbowIndex(poseType)]).ToList();
        List<Vector3> allRightShoulders = list.Select(pose => pose[JointExtension.RShoulderIndex(poseType)]).ToList();
        List<Vector3> allRightWrists = list.Select(pose => pose[JointExtension.RWristIndex(poseType)]).ToList();
        
        float upperArmRightLength = MedianBoneLength(allRightShoulders, allRightElbows);
        float forearmLeftLength = MedianBoneLength(allRightElbows, allRightWrists);
        
        float upperArmLength = (upperArmRightLength + upperArmLengthLeft) / 2;
        float forearmLength = (forearmLeftLength + forearmLengthLeft) / 2;
        
        // deep clone and anchor
        List<List<Vector3>> clone = list.Select(pose => AnchorFinalPose(
            pose, 
            poseType, 
            calfLength, 
            thighLength, 
            upperArmLength, 
            forearmLength)).ToList();

        // smooth
        int jointCount = poseType == PoseType.Coco
            ? Enum.GetNames<CocoJoint>().Length
            : Enum.GetNames<HalpeJoint>().Length;
        for (int i = 0; i < jointCount; i++)
        {
            List<Vector3> jointTimeline = clone.Select(pose => pose[i]).ToList();

            jointTimeline = Transform.MovingAverageSmoothing(jointTimeline, 10);

            // assign smoothed values at joint
            int count = 0;
            foreach (List<Vector3> pose in clone)
            {
                pose[i] = jointTimeline[count];
                count++;
            }
        }

        return clone;
    }

    public void SaveData(string folder)
    {
        List<List<Vector3>> final3DPoseLeadPerFrame = AnchorAndSmooth(merged3DPoseLeadPerFrame);
        List<List<Vector3>> final3DPoseFollowPerFrame = AnchorAndSmooth(merged3DPoseFollowPerFrame);

        string jsonMerged3DPose = JsonConvert.SerializeObject(final3DPoseLeadPerFrame, Formatting.Indented);
        File.WriteAllText(Path.Combine(folder, "figure1.json"), jsonMerged3DPose);

        string jsonMerged3DPoseFollow = JsonConvert.SerializeObject(final3DPoseFollowPerFrame, Formatting.Indented);
        File.WriteAllText(Path.Combine(folder, "figure2.json"), jsonMerged3DPoseFollow);

        Console.WriteLine($"wrote 3d poses to {folder}");
    }
}