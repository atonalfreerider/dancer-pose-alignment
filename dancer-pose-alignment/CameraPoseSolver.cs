using System.Numerics;
using System.Reflection;
using ComputeSharp;
using Newtonsoft.Json;
using SixLabors.ImageSharp;

namespace dancer_pose_alignment;

public enum PoseType
{
    Coco = 0,
    Halpe = 1
}

public class CameraPoseSolver(PoseType poseType)
{
    readonly List<List<Vector3>> merged3DPoseLeadPerFrame = [];
    readonly List<List<Vector3>> merged3DPoseFollowPerFrame = [];

    int poseCount => poseType == PoseType.Coco
        ? Enum.GetNames<CocoJoints>().Length
        : 26; // Halpe

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

    public void PoseFromImage(MemoryStream imageStream, string camName)
    {
        List<List<Vector3>> poses = yolo.CalculatePosesFromImage(imageStream);
        cameras[camName].SetAllPosesAtFrame(poses, frameNumber);
    }

    public List<List<Vector3>> PosesAtFrameAtCamera(string camName)
    {
        return cameras[camName].PosesPerDancerAtFrame(frameNumber);
    }

    public Tuple<int, int> LeadAndFollowIndicesAtCameraAtFrame(string camName)
    {
        return cameras[camName].LeadAndFollowIndexForFrame(frameNumber);
    }

    public Tuple<int, int> MarkDancerAtCam(string camName, Vector2 click, string selectedButton)
    {
        return cameras[camName].MarkDancer(click, frameNumber, selectedButton);
    }
    
    public void MoveKeypointAtCam(string camName, Vector2 click, Tuple<int, int> selectedPoseAndKeypoint)
    {
        cameras[camName].MoveKeypoint(click, frameNumber, selectedPoseAndKeypoint);
    }

    public List<Vector2> ReverseProjectionOfLeadPoseAtCamera(string camName)
    {
        if (merged3DPoseLeadPerFrame.Count <= frameNumber) return [];
        return merged3DPoseLeadPerFrame[frameNumber]
            .Select(x => cameras[camName].ReverseProjectPoint(x, frameNumber)).ToList();
    }

    public List<Vector2> ReverseProjectionOfFollowPoseAtCamera(string camName)
    {
        if (merged3DPoseFollowPerFrame.Count <= frameNumber) return [];
        return merged3DPoseFollowPerFrame[frameNumber]
            .Select(x => cameras[camName].ReverseProjectPoint(x, frameNumber)).ToList();
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

    #region SET CAM ORIENTATION

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
        if (frameNumber == 0)
        {
            cameras[camName].Home();
        }
    }

    public void MoveCameraRight(string camName, float move)
    {
        cameras[camName].PositionsPerFrame[frameNumber] += cameras[camName].Right(frameNumber) * move;
        if (frameNumber == 0)
        {
            cameras[camName].Home();
        }
    }

    public void MoveCameraUp(string camName, float move)
    {
        cameras[camName].PositionsPerFrame[frameNumber] += cameras[camName].Up(frameNumber) * move;
        if (frameNumber == 0)
        {
            cameras[camName].Home();
        }
    }

    #endregion

    #region ITERATORS

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

    public void SaveData(string folder)
    {
        string jsonMerged3DPose = JsonConvert.SerializeObject(merged3DPoseLeadPerFrame, Formatting.Indented);
        File.WriteAllText(Path.Combine(folder, "merged3DPoseLead.json"), jsonMerged3DPose);

        string jsonMerged3DPoseFollow = JsonConvert.SerializeObject(merged3DPoseFollowPerFrame, Formatting.Indented);
        File.WriteAllText(Path.Combine(folder, "merged3DPoseFollow.json"), jsonMerged3DPoseFollow);

        Console.WriteLine($"wrote 3d poses to {folder}");
    }

    #endregion

    public float Calculate3DPosesAndTotalError()
    {
        foreach (CameraSetup cameraSetup in cameras.Values)
        {
            cameraSetup.Project(frameNumber);
        }

        if (merged3DPoseLeadPerFrame.Count <= frameNumber)
        {
            merged3DPoseLeadPerFrame.Add([]);
            merged3DPoseFollowPerFrame.Add([]);
        }

        merged3DPoseLeadPerFrame[frameNumber].Clear();
        merged3DPoseFollowPerFrame[frameNumber].Clear();

        List<float3> leadRayOriginPerCameraPerJoint = []; // in batches of camera count
        List<float3> leadRayDirectionPerCameraPerJoint = []; // in batches of camera count
        List<float> leadJointConfidencePerCameraPerJoint = []; // in batches of camera count
        List<float3> followRayOriginPerCameraPerJoint = []; // in batches of camera count
        List<float3> followRayDirectionPerCameraPerJoint = []; // in batches of camera count
        List<float> followJointConfidencePerCameraPerJoint = []; // in batches of camera count

        int arrayLength = poseCount * cameras.Count;

        for (int i = 0; i < poseCount; i++)
        {
            foreach (CameraSetup cameraSetup in cameras.Values)
            {
                if (cameraSetup.HasPoseAtFrame(frameNumber, true))
                {
                    Ray ray = cameraSetup.PoseRay(frameNumber, i, true);
                    leadRayOriginPerCameraPerJoint.Add(ray.Origin);
                    leadRayDirectionPerCameraPerJoint.Add(ray.Direction);
                    leadJointConfidencePerCameraPerJoint.Add(cameraSetup.JointConfidence(frameNumber, i, true));
                }
                else
                {
                    leadRayOriginPerCameraPerJoint.Add(float3.Zero);
                    leadRayDirectionPerCameraPerJoint.Add(float3.UnitZ);
                    leadJointConfidencePerCameraPerJoint.Add(0);
                }

                if (cameraSetup.HasPoseAtFrame(frameNumber, false))
                {
                    Ray ray = cameraSetup.PoseRay(frameNumber, i, false);
                    followRayOriginPerCameraPerJoint.Add(ray.Origin);
                    followRayDirectionPerCameraPerJoint.Add(ray.Direction);
                    followJointConfidencePerCameraPerJoint.Add(cameraSetup.JointConfidence(frameNumber, i, false));
                }
                else
                {
                    followRayOriginPerCameraPerJoint.Add(float3.Zero);
                    followRayDirectionPerCameraPerJoint.Add(float3.UnitZ);
                    followJointConfidencePerCameraPerJoint.Add(0);
                }
            }
        }

        float3[] leadJointMidpoints = new float3[poseCount];

        using ReadWriteBuffer<float3> leadMinMidpointBuffer =
            GraphicsDevice.GetDefault().AllocateReadWriteBuffer(leadJointMidpoints);
        using ReadOnlyBuffer<float3> leadRayOriginBuffer =
            GraphicsDevice.GetDefault().AllocateReadOnlyBuffer(leadRayOriginPerCameraPerJoint.ToArray());
        using ReadOnlyBuffer<float3> leadRayDirectionBuffer = GraphicsDevice.GetDefault()
            .AllocateReadOnlyBuffer(leadRayDirectionPerCameraPerJoint.ToArray());
        using ReadOnlyBuffer<float> leadJointConfidenceBuffer = GraphicsDevice.GetDefault()
            .AllocateReadOnlyBuffer(leadJointConfidencePerCameraPerJoint.ToArray());

        MidpointFinder leadMidpointFinder = new MidpointFinder(
            leadMinMidpointBuffer,
            leadRayOriginBuffer,
            leadRayDirectionBuffer,
            leadJointConfidenceBuffer,
            cameras.Count);

        try
        {
            GraphicsDevice.GetDefault().For(arrayLength, leadMidpointFinder);
        }
        catch (TargetInvocationException e)
        {
            Console.WriteLine(e);
        }

        float3[] leadResults = leadMinMidpointBuffer.ToArray();
        foreach (float3 result in leadResults)
        {
            merged3DPoseLeadPerFrame[frameNumber].Add(new Vector3(result.X, result.Y, result.Z));
        }

        float3[] followJointMidpoints = new float3[poseCount];

        using ReadWriteBuffer<float3> followMinMidpointBuffer =
            GraphicsDevice.GetDefault().AllocateReadWriteBuffer(followJointMidpoints);
        using ReadOnlyBuffer<float3> followRayOriginBuffer = GraphicsDevice.GetDefault()
            .AllocateReadOnlyBuffer(followRayOriginPerCameraPerJoint.ToArray());
        using ReadOnlyBuffer<float3> followRayDirectionBuffer = GraphicsDevice.GetDefault()
            .AllocateReadOnlyBuffer(followRayDirectionPerCameraPerJoint.ToArray());
        using ReadOnlyBuffer<float> followJointConfidenceBuffer = GraphicsDevice.GetDefault()
            .AllocateReadOnlyBuffer(followJointConfidencePerCameraPerJoint.ToArray());

        MidpointFinder followMidpointFinder = new MidpointFinder(
            followMinMidpointBuffer,
            followRayOriginBuffer,
            followRayDirectionBuffer,
            followJointConfidenceBuffer,
            cameras.Count);

        try
        {
            GraphicsDevice.GetDefault().For(arrayLength, followMidpointFinder);
        }
        catch (TargetInvocationException e)
        {
            Console.WriteLine(e);
        }

        float3[] followResults = followMinMidpointBuffer.ToArray();
        foreach (float3 result in followResults)
        {
            merged3DPoseFollowPerFrame[frameNumber].Add(new Vector3(result.X, result.Y, result.Z));
        }

        float totalError = 0;
        foreach (CameraSetup cameraSetup in cameras.Values)
        {
            totalError += cameraSetup.Error(merged3DPoseLeadPerFrame[frameNumber],
                merged3DPoseFollowPerFrame[frameNumber], frameNumber);
        }

        return totalError;
    }

    static float Confidence(IEnumerable<Vector3> lead, IEnumerable<Vector3> follow)
    {
        return lead.Sum(vector3 => vector3.Z) + follow.Sum(vector3 => vector3.Z);
    }

    public bool AnyPointsBelowGround() => merged3DPoseLeadPerFrame[frameNumber].Any(vec => vec.Y < -.1f) ||
                                          merged3DPoseFollowPerFrame[frameNumber].Any(vec => vec.Y < -.1f);

    public bool LowestLeadAnkleIsMoreThan10CMAboveZero()
    {
        float lowestLeadAnkle = Math.Min(
            merged3DPoseLeadPerFrame[frameNumber][
                poseType == PoseType.Coco ? (int)CocoJoints.L_Ankle : (int)HalpeJoints.LAnkle].Y,
            merged3DPoseLeadPerFrame[frameNumber][
                poseType == PoseType.Coco ? (int)CocoJoints.R_Ankle : (int)HalpeJoints.RAnkle].Y);
        return lowestLeadAnkle > 0.1f; // 10cm above ground
    }

    public bool AnyPointsHigherThan2pt5Meters()
    {
        return merged3DPoseLeadPerFrame[frameNumber].Any(vec => vec.Y > 2.5) ||
               merged3DPoseFollowPerFrame[frameNumber].Any(vec => vec.Y > 2.5);
    }
}