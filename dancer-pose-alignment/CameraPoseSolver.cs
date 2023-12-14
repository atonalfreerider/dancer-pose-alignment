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
    double timeOffset;

    Yolo yolo = new("yolov8x-pose.onnx"); // this is in the assembly dir 

    public void CreateAndPlaceCamera(
        string name,
        Vector2 imageSize,
        int frameCount)
    {
        CameraSetup camera = new(imageSize, frameCount, poseType);

        Quaternion centerLook = Transform.LookAt(
            new Vector3(0, 1.5f, 0),
            Quaternion.Identity,
            camera.PositionsPerFrame[0]);
        camera.RotationsPerFrame[0] = centerLook;

        cameras.Add(name, camera);
    }

    public void SetPoseFromImage(MemoryStream imageStream, string camName)
    {
        List<List<Vector3>> poses = yolo.CalculatePosesFromImage(imageStream);
        cameras[camName].SetAllPosesAtFrame(poses, frameNumber);
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
            cameraSetup.CopyPositionsToNextFrame(frameNumber);
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
        return cameras[camName].MarkDancer(click, frameNumber, selectedButton);
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
            return merged3DPoseLeadPerFrame[frameNumber]
                .Select(vec => cameras[camName].ReverseProjectPoint(vec, frameNumber)).ToList();
        }

        if (merged3DPoseFollowPerFrame.Count <= frameNumber) return [];
        return merged3DPoseFollowPerFrame[frameNumber]
            .Select(vec => cameras[camName].ReverseProjectPoint(vec, frameNumber)).ToList();
    }

    public List<Vector2> ReverseProjectOriginCrossAtCamera(string camName)
    {
        CameraSetup cameraSetup = cameras[camName];

        return
        [
            cameraSetup.ReverseProjectPoint(Vector3.Zero, frameNumber),
            cameraSetup.ReverseProjectPoint(Vector3.UnitX, frameNumber),
            cameraSetup.ReverseProjectPoint(Vector3.UnitY, frameNumber),
            cameraSetup.ReverseProjectPoint(Vector3.UnitZ, frameNumber)
        ];
    }

    public void SaveData(string folder)
    {
        string jsonMerged3DPose = JsonConvert.SerializeObject(merged3DPoseLeadPerFrame, Formatting.Indented);
        File.WriteAllText(Path.Combine(folder, "figure1.json"), jsonMerged3DPose);

        string jsonMerged3DPoseFollow = JsonConvert.SerializeObject(merged3DPoseFollowPerFrame, Formatting.Indented);
        File.WriteAllText(Path.Combine(folder, "figure2.json"), jsonMerged3DPoseFollow);

        Console.WriteLine($"wrote 3d poses to {folder}");
    }

    #region ITERATORS

    public bool AreAllCamerasOriented()
    {
        return !cameras.Values.Any(cam =>
            cam.LeadAndFollowIndexForFrame(frameNumber).Item1 == -1 ||
            cam.LeadAndFollowIndexForFrame(frameNumber).Item2 == -1);
    }

    public void TryHomeCamera(string camName)
    {
        cameras[camName].Home();
    }

    public void IterationLoop()
    {
        if (merged3DPoseFollowPerFrame.Count <= frameNumber) return;
        float totalError = Calculate3DPosesAndTotalError();

        int iterationCount = 0;
        while (iterationCount < 10000)
        {
            CameraSetup highestErrorCam = null;
            float highestError = 0;
            foreach (CameraSetup cameraSetup in cameras.Values)
            {
                float error = cameraSetup.Error(
                    merged3DPoseLeadPerFrame[frameNumber],
                    merged3DPoseFollowPerFrame[frameNumber],
                    frameNumber);

                if (error > highestError)
                {
                    highestError = error;
                    highestErrorCam = cameraSetup;
                }
            }

            if (highestErrorCam == null)
            {
                Console.WriteLine("couldn't find highest error camera");
                break;
            }

            // TODO translate XYZ +/- .1m and determine which path has lowest error after homing
            bool moved = highestErrorCam.IterateOrientation(this, frameNumber);
            if (!moved)
            {
                break;
            }

            totalError = Calculate3DPosesAndTotalError();
            Console.WriteLine($"{iterationCount}:{totalError}");
            iterationCount++;
        }

        totalError = Calculate3DPosesAndTotalError();
        Console.WriteLine(totalError);
    }

    public float Calculate3DPosesAndTotalError()
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

        return cameras.Values.Sum(cameraSetup =>
            cameraSetup.Error(
                merged3DPoseLeadPerFrame[frameNumber],
                merged3DPoseFollowPerFrame[frameNumber], frameNumber));
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

        return minMidpointBuffer.ToArray()
            .Select(result => new Vector3(result.X, result.Y, result.Z))
            .ToList();
    }

    #endregion

    #region CAMERA MOTION

    public void YawCamera(string camName, float angle)
    {
        cameras[camName].RotationsPerFrame[frameNumber] *= Quaternion.CreateFromAxisAngle(Vector3.UnitY, angle);
    }

    public void PitchCamera(string camName, float angle)
    {
        cameras[camName].RotationsPerFrame[frameNumber] *= Quaternion.CreateFromAxisAngle(Vector3.UnitX, angle);
    }

    public void RollCamera(string camName, float angle)
    {
        cameras[camName].RotationsPerFrame[frameNumber] *= Quaternion.CreateFromAxisAngle(Vector3.UnitZ, angle);
    }

    public void ZoomCamera(string camName, float zoom)
    {
        cameras[camName].FocalLength += zoom;
    }

    public void MoveCameraForward(string camName, float move)
    {
        cameras[camName].PositionsPerFrame[frameNumber] += cameras[camName].Forward(frameNumber) * move;
    }

    public void MoveCameraRight(string camName, float move)
    {
        cameras[camName].PositionsPerFrame[frameNumber] += cameras[camName].Right(frameNumber) * move;
    }

    public void MoveCameraUp(string camName, float move)
    {
        cameras[camName].PositionsPerFrame[frameNumber] += cameras[camName].Up(frameNumber) * move;
    }

    #endregion

    #region TEST POINT

    public bool AnyPointsBelowGround() => merged3DPoseLeadPerFrame[frameNumber].Any(vec => vec.Y < -.1f) ||
                                          merged3DPoseFollowPerFrame[frameNumber].Any(vec => vec.Y < -.1f);

    public bool LowestLeadAnkleIsMoreThan10CMAboveZero()
    {
        float lowestLeadAnkle = Math.Min(
            merged3DPoseLeadPerFrame[frameNumber][
                poseType == PoseType.Coco ? (int)CocoJoint.L_Ankle : (int)HalpeJoint.LAnkle].Y,
            merged3DPoseLeadPerFrame[frameNumber][
                poseType == PoseType.Coco ? (int)CocoJoint.R_Ankle : (int)HalpeJoint.RAnkle].Y);
        return lowestLeadAnkle > 0.1f; // 10cm above ground
    }

    public bool AnyPointsHigherThan2pt5Meters()
    {
        return merged3DPoseLeadPerFrame[frameNumber].Any(vec => vec.Y > 2.5) ||
               merged3DPoseFollowPerFrame[frameNumber].Any(vec => vec.Y > 2.5);
    }

    #endregion
}