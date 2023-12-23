using System.Numerics;
using System.Reflection;
using ComputeSharp;
using Newtonsoft.Json;

namespace dancer_pose_alignment;

public class CameraPoseSolver(PoseType poseType)
{
    readonly List<List<Vector3>> merged3DPoseLeadPerFrame = [];
    readonly List<List<Vector3>> merged3DPoseFollowPerFrame = [];

    readonly Dictionary<string, CameraSetup> cameras = [];
    int frameNumber = 0;
    public int MaximumFrameCount = int.MaxValue;

    const float leadLegLimbLenth = .4f;
    const float followLegLimbLength = .4f;

    const float leadShoulderHipArmLength = .3f;
    const float followShoulderHipArmLength = .28f;

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

    readonly Yolo yolo = new("yolov8x-pose.onnx"); // this is in the assembly dir 

    public void CreateCamera(
        string name,
        Vector2 imageSize,
        int frameCount,
        int startingFrame)
    {
        CameraSetup camera = new(name, imageSize, frameCount, poseType, startingFrame);
        cameras.Add(name, camera);
    }

    public void SetPoseFromImage(MemoryStream imageStream, string camName)
    {
        List<List<Vector3>> poses = yolo.CalculatePosesFromImage(imageStream);
        cameras[camName].SetAllPosesAtFrame(poses, frameNumber);

        if (frameNumber == 0)
        {
            TryHomeCamera(camName);
        }
    }

    public void SetAllPoses(List<List<List<Vector3>>> posesByFrame, string camName)
    {
        cameras[camName].SetAllPosesForEveryFrame(posesByFrame);
        
        if (frameNumber == 0)
        {
            TryHomeCamera(camName);
        }
    }

    public List<List<Vector3>> PosesAtFrameAtCamera(string camName)
    {
        return cameras[camName].PosesPerDancerAtFrame(frameNumber);
    }

    public bool Advance()
    {
        if (frameNumber >= MaximumFrameCount - 1) return false;

        frameNumber++;
        foreach (CameraSetup cameraSetup in cameras.Values)
        {
            cameraSetup.CopyRotationToNextFrame(frameNumber);
            cameraSetup.Match3DPoseToPoses(frameNumber);
        }

        return true;
    }

    public bool Rewind()
    {
        if (frameNumber <= 0) return false;

        frameNumber--;
        return true;
    }

    public Tuple<int, int> LeadAndFollowIndicesAtCameraAtFrame(string camName)
    {
        return cameras[camName].LeadAndFollowIndexForFrame(frameNumber);
    }

    /// <summary>
    /// For the selected camera, attempt to select the figure from the closes joint
    /// </summary>
    /// <returns>The index of the dancer and the index of the joint</returns>
    public Tuple<int, int> MarkDancerAtCam(string camName, Vector2 click, string selectedButton)
    {
        Tuple<int, int> selectionAndJoint = cameras[camName].MarkDancer(click, frameNumber, selectedButton);
        switch (selectedButton)
        {
            case "Lead":
            case "Follow":
                cameras[camName].CalculateCameraWall(frameNumber);
                break;
        }

        return selectionAndJoint;
    }

    public void MoveKeypointAtCam(string camName, Vector2 click, Tuple<int, int> selectedPoseAndKeypoint)
    {
        cameras[camName].MoveKeypoint(click, frameNumber, selectedPoseAndKeypoint);
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

    public List<Tuple<Vector2, Vector2>> ReverseProjectCameraPositionsAtCameraAndManualPair(string camName)
    {
        List<Tuple<Vector2, Vector2>> pointPairs = [];
        foreach ((string otherCamName, Vector3 camPos) in cameraPositions)
        {
            if (camName == otherCamName) continue;

            Vector2 point = cameras[camName].ReverseProjectPoint(camPos, frameNumber);
            if (cameras[camName].ManualCameraPositionsByFrameByCamName.ContainsKey(otherCamName))
            {
                Vector2 manualPos = cameras[camName].ManualCameraPositionsByFrameByCamName[otherCamName][frameNumber]
                    .ImgPosition;
                pointPairs.Add(new Tuple<Vector2, Vector2>(point, manualPos));
            }
            else
            {
                pointPairs.Add(new Tuple<Vector2, Vector2>(point, new Vector2(-1, -1)));
            }
        }

        return pointPairs;
    }

    public void SaveData(string folder)
    {
        string jsonMerged3DPose = JsonConvert.SerializeObject(merged3DPoseLeadPerFrame, Formatting.Indented);
        File.WriteAllText(Path.Combine(folder, "figure1.json"), jsonMerged3DPose);

        string jsonMerged3DPoseFollow = JsonConvert.SerializeObject(merged3DPoseFollowPerFrame, Formatting.Indented);
        File.WriteAllText(Path.Combine(folder, "figure2.json"), jsonMerged3DPoseFollow);

        Console.WriteLine($"wrote 3d poses to {folder}");
    }
    
    public void UnassignEachIndexAndMatchToClosest()
    {
        CalculateLeadFollow3DPoses();
        foreach (CameraSetup cameraSetup in cameras.Values)
        {
            cameraSetup.Unassign(0);
            cameraSetup.Match3DPoseToPoses(0, 3000);
        }
    }

    #region ITERATORS

    public bool AreAllCamerasOriented()
    {
        return !cameras.Values.Any(cam =>
            cam.LeadAndFollowIndexForFrame(frameNumber).Item1 == -1 ||
            cam.LeadAndFollowIndexForFrame(frameNumber).Item2 == -1);
    }

    public void HomeAllCameras()
    {
        if (frameNumber > 0) return;
        foreach (CameraSetup cam in cameras.Values)
        {
            cam.Home();
        }
    }
    
    void TryHomeCamera(string camName)
    {
        if (frameNumber > 0) return;
        cameras[camName].Home();
    }

    public void SetCamR()
    {
        List<Tuple<float, float>> cameraWall = OrderedAndSmoothedCameraWall();
        foreach (CameraSetup camerasValue in cameras.Values)
        {
            camerasValue.SetRadiusFromCameraWall(cameraWall);
        }
    }

    public void SetCameraHeights()
    {
        if (frameNumber > 0) return;

        foreach (CameraSetup cam in cameras.Values)
        {
            cam.HeightsSetByOtherCameras.Clear();
        }

        foreach (CameraSetup cam in cameras.Values)
        {
            foreach ((string otherCamName, List<CameraSetup.CameraHandAnchor> cameraHandAnchorList) in cam
                         .ManualCameraPositionsByFrameByCamName)
            {
                CameraSetup.CameraHandAnchor cameraHandAnchor = cameraHandAnchorList[0];
                float poseHeight = cam.PoseHeight(cameraHandAnchor.PoseIndex, cameraHandAnchor.ImgPosition.Y);

                CameraSetup otherCam = cameras[otherCamName];
                otherCam.HeightsSetByOtherCameras.Add(poseHeight);
            }
        }
    }

    public void CalculateLeadFollow3DPoses()
    {
        foreach (CameraSetup cameraSetup in cameras.Values)
        {
            cameraSetup.Project(frameNumber);
        }

        if (merged3DPoseLeadPerFrame.Count <= frameNumber)
        {
            // add a new frame
            merged3DPoseLeadPerFrame.Add([]);
        }

        if (merged3DPoseFollowPerFrame.Count <= frameNumber)
        {
            merged3DPoseFollowPerFrame.Add([]);
        }

        merged3DPoseLeadPerFrame[frameNumber].Clear();
        merged3DPoseFollowPerFrame[frameNumber].Clear();

        List<Vector3> merged3DPoseLead = Calculate3DPose(true);
        List<Vector3> merged3DPoseFollow = Calculate3DPose(false);

        merged3DPoseLeadPerFrame[frameNumber] = merged3DPoseLead;
        merged3DPoseFollowPerFrame[frameNumber] = merged3DPoseFollow;

        foreach (CameraSetup cameraSetup in cameras.Values)
        {
            cameraSetup.CurrentLead3DPose = merged3DPoseLead;
            cameraSetup.CurrentFollow3DPose = merged3DPoseFollow;
        }
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

        MidpointFinder midpointFinder = new MidpointFinder(
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

        List<Vector3> final3DPose = [];
        for (int i = 0; i < JointExtension.PoseCount(poseType); i++)
        {
            final3DPose.Add(Vector3.Zero);
        }


        // ANCHOR LEGS
        Vector3 rAnklePos = isLead && frameNumber == 0
            ? Vector3.Zero // root to origin
            : scatterPose[JointExtension.RAnkleIndex(poseType)];
        Vector3 lAnklePos = isLead && frameNumber == 0
            ? new Vector3(-leadShoulderHipArmLength, 0, 0) // root to stance
            : scatterPose[JointExtension.LAnkleIndex(poseType)];

        Vector3 rKneePos = scatterPose[JointExtension.RKneeIndex(poseType)];
        Vector3 lKneePos = scatterPose[JointExtension.LKneeIndex(poseType)];

        Vector3 rHipPos = scatterPose[JointExtension.RHipIndex(poseType)];
        Vector3 lHipPos = scatterPose[JointExtension.LHipIndex(poseType)];

        if (rAnklePos.Y < lAnklePos.Y)
        {
            // right ankle is lower to the ground
            final3DPose[JointExtension.RAnkleIndex(poseType)] = rAnklePos with { Y = 0 };
            rKneePos = rAnklePos + Vector3.Normalize(rKneePos - rAnklePos) *
                (isLead ? leadLegLimbLenth : followLegLimbLength);
            final3DPose[JointExtension.RKneeIndex(poseType)] = rKneePos;
            rHipPos = rKneePos + Vector3.Normalize(rHipPos - rKneePos) *
                (isLead ? leadLegLimbLenth : followLegLimbLength);
            final3DPose[JointExtension.RHipIndex(poseType)] = rHipPos;

            if (frameNumber == 0)
            {
                final3DPose[JointExtension.LAnkleIndex(poseType)] = lAnklePos with { Y = 0 };
                lKneePos = lAnklePos + Vector3.Normalize(lKneePos - lAnklePos) *
                    (isLead ? leadLegLimbLenth : followLegLimbLength);
                final3DPose[JointExtension.LKneeIndex(poseType)] = lKneePos;
                lHipPos = lKneePos + Vector3.Normalize(lHipPos - lKneePos) *
                    (isLead ? leadLegLimbLenth : followLegLimbLength);
                final3DPose[JointExtension.LHipIndex(poseType)] = lHipPos;
            }
            else
            {
                lHipPos = rHipPos + Vector3.Normalize(lHipPos - rHipPos) *
                    (isLead ? leadShoulderHipArmLength : followShoulderHipArmLength);
                final3DPose[JointExtension.LHipIndex(poseType)] = lHipPos;
                lKneePos = lHipPos +
                           Vector3.Normalize(lKneePos - lHipPos) * (isLead ? leadLegLimbLenth : followLegLimbLength);
                final3DPose[JointExtension.LKneeIndex(poseType)] = lKneePos;
                lAnklePos = lKneePos + Vector3.Normalize(lAnklePos - lKneePos) * (isLead
                    ? leadLegLimbLenth
                    : followLegLimbLength);
                final3DPose[JointExtension.LAnkleIndex(poseType)] = lAnklePos;
            }
        }
        else
        {
            final3DPose[JointExtension.LAnkleIndex(poseType)] = lAnklePos;
            lKneePos = lAnklePos + Vector3.Normalize(lKneePos - lAnklePos) * (isLead
                ? leadLegLimbLenth
                : followLegLimbLength);
            final3DPose[JointExtension.LKneeIndex(poseType)] = lKneePos;
            lHipPos = lKneePos +
                      Vector3.Normalize(lHipPos - lKneePos) * (isLead ? leadLegLimbLenth : followLegLimbLength);
            final3DPose[JointExtension.LHipIndex(poseType)] = lHipPos;

            if (frameNumber == 0)
            {
                final3DPose[JointExtension.RAnkleIndex(poseType)] = rAnklePos with { Y = 0 };
                rKneePos = rAnklePos + Vector3.Normalize(rKneePos - rAnklePos) *
                    (isLead ? leadLegLimbLenth : followLegLimbLength);
                final3DPose[JointExtension.RKneeIndex(poseType)] = rKneePos;
                rHipPos = rKneePos + Vector3.Normalize(rHipPos - rKneePos) *
                    (isLead ? leadLegLimbLenth : followLegLimbLength);
                final3DPose[JointExtension.RHipIndex(poseType)] = rHipPos;
            }
            else
            {
                rHipPos = lHipPos + Vector3.Normalize(rHipPos - lHipPos) *
                    (isLead ? leadShoulderHipArmLength : followShoulderHipArmLength);
                final3DPose[JointExtension.RHipIndex(poseType)] = rHipPos;
                rKneePos = rHipPos +
                           Vector3.Normalize(rKneePos - rHipPos) * (isLead ? leadLegLimbLenth : followLegLimbLength);
                final3DPose[JointExtension.RKneeIndex(poseType)] = rKneePos;
                rAnklePos = rKneePos + Vector3.Normalize(rAnklePos - rKneePos) * (isLead
                    ? leadLegLimbLenth
                    : followLegLimbLength);
                final3DPose[JointExtension.RAnkleIndex(poseType)] = rAnklePos;
            }
        }

        // ANCHOR ARMS
        Vector3 rShoulderPos = scatterPose[JointExtension.RShoulderIndex(poseType)];
        Vector3 lShoulderPos = scatterPose[JointExtension.LShoulderIndex(poseType)];

        Vector3 rElbowPos = scatterPose[JointExtension.RElbowIndex(poseType)];
        Vector3 lElbowPos = scatterPose[JointExtension.LElbowIndex(poseType)];

        Vector3 rWristPos = scatterPose[JointExtension.RWristIndex(poseType)];
        Vector3 lWristPos = scatterPose[JointExtension.LWristIndex(poseType)];

        final3DPose[JointExtension.RShoulderIndex(poseType)] = rShoulderPos;
        rElbowPos = rShoulderPos + Vector3.Normalize(rElbowPos - rShoulderPos) *
            (isLead ? leadShoulderHipArmLength : followShoulderHipArmLength);
        final3DPose[JointExtension.RElbowIndex(poseType)] = rElbowPos;
        rWristPos = rElbowPos + Vector3.Normalize(rWristPos - rElbowPos) *
            (isLead ? leadShoulderHipArmLength : followShoulderHipArmLength);
        final3DPose[JointExtension.RWristIndex(poseType)] = rWristPos;

        final3DPose[JointExtension.LShoulderIndex(poseType)] = lShoulderPos;
        lElbowPos = lShoulderPos + Vector3.Normalize(lElbowPos - lShoulderPos) *
            (isLead ? leadShoulderHipArmLength : followShoulderHipArmLength);
        final3DPose[JointExtension.LElbowIndex(poseType)] = lElbowPos;
        lWristPos = lElbowPos + Vector3.Normalize(lWristPos - lElbowPos) *
            (isLead ? leadShoulderHipArmLength : followShoulderHipArmLength);
        final3DPose[JointExtension.LWristIndex(poseType)] = lWristPos;

        final3DPose[JointExtension.NoseIndex(poseType)] = scatterPose[JointExtension.NoseIndex(poseType)];
        final3DPose[JointExtension.LEarIndex(poseType)] = scatterPose[JointExtension.LEarIndex(poseType)];
        final3DPose[JointExtension.REarIndex(poseType)] = scatterPose[JointExtension.REarIndex(poseType)];
        final3DPose[JointExtension.LEyeIndex(poseType)] = scatterPose[JointExtension.LEyeIndex(poseType)];
        final3DPose[JointExtension.REyeIndex(poseType)] = scatterPose[JointExtension.REyeIndex(poseType)];

        return final3DPose;
    }

    #endregion

    List<Tuple<float, float>> OrderedAndSmoothedCameraWall()
    {   
        List<Tuple<float,float>> ordered = cameras.Values
            .SelectMany(cam => cam.CameraWall).OrderBy(x => x.Item1).ToList();
        List<Vector3> allPoints = [];
        foreach (Tuple<float,float> tuple in ordered)
        {
            Vector3 point = new Vector3(tuple.Item2 * MathF.Sin(tuple.Item1), 0, tuple.Item2 * MathF.Cos(tuple.Item1));
            allPoints.Add(point);
        }

        allPoints = Transform.MovingAverageSmoothing(allPoints, 4);
        
        List<Tuple<float, float>> final = [];
        for(int i = 0; i < allPoints.Count; i++)
        {
            Vector3 point = allPoints[i];
            float rad = Vector3.Distance(Vector3.Zero, point);
            final.Add(new Tuple<float, float>(ordered[i].Item1, rad));
        }

        return final;
    }

}