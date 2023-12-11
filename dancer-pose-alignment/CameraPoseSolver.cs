using System.ComponentModel;
using System.Numerics;
using System.Reflection;
using ComputeSharp;
using Newtonsoft.Json;

namespace dancer_pose_alignment;

public class CameraPoseSolver
{
    readonly List<List<Vector3>> merged3DPoseLeadPerFrame = [];
    readonly List<List<Vector3>> merged3DPoseFollowPerFrame = [];

    readonly int poseCount = 26; // Halpe
    readonly List<CameraSetup> cameras = [];
    int frameNumber = 0;
    int totalFrameCount = 0;

    public void LoadPoses(string directoryPath)
    {
        List<Vector3> cameraPositions = [];
        List<Vector2> cameraSizes = [];

        Dictionary<int, List<List<Vector3>>> allLeadPosesByCamera = [];
        Dictionary<int, List<List<Vector3>>> allFollowPosesByCamera = [];
        Dictionary<int, List<List<Vector3>>> allMirrorLeadPosesByCamera = [];
        Dictionary<int, List<List<Vector3>>> allMirrorFollowPosesByCamera = [];

        Dictionary<int, List<List<Vector3>>> allOtherCameraPositionsByFrameByCamera = [];
        Dictionary<int, List<List<Vector3>>> mirrorAllOtherCameraPositionsByFrameByCamera = [];


        foreach (string file in Directory.GetFiles(directoryPath, "*.json"))
        {
            string fileName = Path.GetFileNameWithoutExtension(file);
            if (fileName.StartsWith("lead") ||
                fileName.StartsWith("follow") ||
                fileName.StartsWith("mirror-lead") ||
                fileName.StartsWith("mirror-follow"))
            {
                string jsonContent = File.ReadAllText(file);
                List<List<Vector3>> dancerPosesByFrames =
                    JsonConvert.DeserializeObject<List<List<Vector3>>>(jsonContent);
                totalFrameCount = dancerPosesByFrames.Count;

                int associatedCamera = int.Parse(fileName[^1].ToString());
                if (fileName.StartsWith("lead"))
                {
                    allLeadPosesByCamera.Add(associatedCamera, dancerPosesByFrames);
                }
                else if (fileName.StartsWith("follow"))
                {
                    allFollowPosesByCamera.Add(associatedCamera, dancerPosesByFrames);
                }
                else if (fileName.StartsWith("mirror-lead"))
                {
                    allMirrorLeadPosesByCamera.Add(associatedCamera, dancerPosesByFrames);
                }
                else if (fileName.StartsWith("mirror-follow"))
                {
                    allMirrorFollowPosesByCamera.Add(associatedCamera, dancerPosesByFrames);
                }
            }
            else
            {
                if (fileName.StartsWith("camera-positions"))
                {
                    string jsonContent = File.ReadAllText(file);
                    cameraPositions = JsonConvert.DeserializeObject<List<Vector3>>(jsonContent);

                    // rescale and offset the camera Z and X, so that the origin is in the middle of the 10m x 8m dance floor. Also 1000 pixels = 10M
                    for (int i = 0; i < cameraPositions.Count; i++)
                    {
                        cameraPositions[i] = cameraPositions[i] with
                        {
                            X = cameraPositions[i].X / 100 - 5,
                            Z = -(cameraPositions[i].Z / 100 - 4) // orient forward
                        };
                    }
                }
                else if (fileName.StartsWith("cameraSizes"))
                {
                    string jsonContent = File.ReadAllText(file);
                    cameraSizes = JsonConvert.DeserializeObject<List<Vector2>>(jsonContent);
                }
                else
                {
                    int associatedCamera = int.Parse(fileName[^1].ToString());
                    List<List<Vector3>> otherCameraPositionsByFrame =
                        JsonConvert.DeserializeObject<List<List<Vector3>>>(File.ReadAllText(file));
                    if (fileName.StartsWith("mirror"))
                    {
                        mirrorAllOtherCameraPositionsByFrameByCamera.Add(associatedCamera, otherCameraPositionsByFrame);
                    }
                    else
                    {
                        allOtherCameraPositionsByFrameByCamera.Add(associatedCamera, otherCameraPositionsByFrame);
                    }
                }
            }
        }

        // seed
        for (int i = 0; i < totalFrameCount; i++)
        {
            merged3DPoseLeadPerFrame.Add([]);
            merged3DPoseFollowPerFrame.Add([]);
        }

        CreateAndPlaceCameras(cameraSizes, cameraPositions);
        AddPosesToCamera(
            allLeadPosesByCamera,
            allFollowPosesByCamera,
            allMirrorLeadPosesByCamera,
            allMirrorFollowPosesByCamera,
            allOtherCameraPositionsByFrameByCamera,
            mirrorAllOtherCameraPositionsByFrameByCamera);
    }

    public List<List<List<Vector3>>> AllPosesAtFramePerCamera()
    {
        return cameras.Select(camera => camera.PosesPerDancerAtFrame(frameNumber).ToList()).ToList();
    }

    public List<List<Vector3>> AllVisibleCamerasAtFramePerCamera()
    {
        return cameras.Select(camera => camera.OtherCamerasPositionAndConfidenceAtFrame(frameNumber).ToList()).ToList();
    }

    public List<List<Vector3>> AllMirrorVisibleCamerasAtFramePerCamera()
    {
        return cameras.Select(camera => camera.OtherMirrorCamerasPositionAndConfidenceAtFrame(frameNumber).ToList())
            .ToList();
    }

    public List<List<Vector2>> ReverseProjectionOfLeadPosePerCamera()
    {
        return cameras.Select(camera => merged3DPoseLeadPerFrame[frameNumber]
                .Select(x => camera.ReverseProjectPoint(x, frameNumber)).ToList())
            .ToList();
    }

    public List<List<Vector2>> ReverseProjectionOfFollowPosePerCamera()
    {
        return cameras.Select(camera => merged3DPoseFollowPerFrame[frameNumber]
                .Select(x => camera.ReverseProjectPoint(x, frameNumber)).ToList())
            .ToList();
    }

    public List<List<Vector2>> ReverseProjectOriginCrossPerCamera()
    {
        List<List<Vector2>> originCross = [];
        foreach (CameraSetup camera in cameras)
        {
            originCross.Add([
                camera.ReverseProjectPoint(Vector3.Zero, frameNumber),
                camera.ReverseProjectPoint(Vector3.UnitX, frameNumber),
                camera.ReverseProjectPoint(Vector3.UnitY, frameNumber),
                camera.ReverseProjectPoint(Vector3.UnitZ, frameNumber)
            ]);
        }

        return originCross;
    }

    public List<Vector2> ReverseProjectionsOfOtherCamerasPerCamera(int cameraIndex)
    {
        return cameras.Select((camera, index) => index == cameraIndex
            ? Vector2.Zero
            : cameras[cameraIndex].ReverseProjectPoint(camera.PositionsPerFrame[frameNumber], frameNumber)).ToList();
    }

    void CreateAndPlaceCameras(
        IReadOnlyList<Vector2> imageSizes,
        IReadOnlyList<Vector3> seedCameraPositions)
    {
        const int testingFrameNumber = 0;
        for (int i = 0; i < imageSizes.Count; i++)
        {
            CameraSetup camera = new()
            {
                Size = imageSizes[i]
            };

            if (seedCameraPositions.Count != 0)
            {
                camera.PositionsPerFrame.Add(seedCameraPositions[i]);
            }
            else
            {
                float angle = -(float)i / imageSizes.Count * 2 * MathF.PI; // ccw
                const float radius = 3f;
                const float startingHeight = 1.5f;
                camera.PositionsPerFrame.Add(new Vector3(
                    MathF.Sin(angle) * radius,
                    startingHeight,
                    MathF.Cos(angle) * radius));
            }

            Quaternion centerLook = Transform.LookAt(
                new Vector3(0, 1.5f, 0),
                Quaternion.Identity,
                camera.PositionsPerFrame[testingFrameNumber]);
            camera.RotationsPerFrame.Add(centerLook);

            cameras.Add(camera);
        }
    }

    void AddPosesToCamera(
        IReadOnlyDictionary<int, List<List<Vector3>>> allLeadPosesByCamera,
        IReadOnlyDictionary<int, List<List<Vector3>>> allFollowPosesByCamera,
        IReadOnlyDictionary<int, List<List<Vector3>>> allMirrorLeadPosesByCamera,
        IReadOnlyDictionary<int, List<List<Vector3>>> allMirrorFollowPosesByCamera,
        IReadOnlyDictionary<int, List<List<Vector3>>> allOtherCameraPositionsByFrameByCamera,
        IReadOnlyDictionary<int, List<List<Vector3>>> mirrorAllOtherCameraPositionsByFrameByCamera)
    {
        int count = 0;
        foreach (CameraSetup camera in cameras)
        {
            camera.AddPosesAndRecenterAndScaleToCamera(
                allLeadPosesByCamera[count],
                allFollowPosesByCamera[count],
                allMirrorLeadPosesByCamera[count],
                allMirrorFollowPosesByCamera[count],
                allOtherCameraPositionsByFrameByCamera[count],
                mirrorAllOtherCameraPositionsByFrameByCamera[count]);
            count++;
        }
    }

    #region SET CAM ORIENTATION

    public void HomeAllCameras()
    {
        foreach (CameraSetup camera in cameras)
        {
            camera.Home();
        }
    }

    public void YawCamera(int camIndex, float angle)
    {
        cameras[camIndex].RotationsPerFrame[frameNumber] *= Quaternion.CreateFromAxisAngle(Vector3.UnitY, angle);
    }

    public void PitchCamera(int camIndex, float angle)
    {
        cameras[camIndex].RotationsPerFrame[frameNumber] *= Quaternion.CreateFromAxisAngle(Vector3.UnitX, angle);
    }

    public void RollCamera(int camIndex, float angle)
    {
        cameras[camIndex].RotationsPerFrame[frameNumber] *= Quaternion.CreateFromAxisAngle(Vector3.UnitZ, angle);
    }

    public void ZoomCamera(int camIndex, float zoom)
    {
        cameras[camIndex].FocalLength += zoom;
    }

    public void MoveCameraForward(int camIndex, float move)
    {
        cameras[camIndex].PositionsPerFrame[frameNumber] += cameras[camIndex].Forward(frameNumber) * move;
        if (frameNumber == 0)
        {
            cameras[camIndex].Home();
        }
    }

    public void MoveCameraRight(int camIndex, float move)
    {
        cameras[camIndex].PositionsPerFrame[frameNumber] += cameras[camIndex].Right(frameNumber) * move;
        if (frameNumber == 0)
        {
            cameras[camIndex].Home();
        }
    }

    public void MoveCameraUp(int camIndex, float move)
    {
        cameras[camIndex].PositionsPerFrame[frameNumber] += cameras[camIndex].Up(frameNumber) * move;
        if (frameNumber == 0)
        {
            cameras[camIndex].Home();
        }
    }

    #endregion

    #region ITERATORS

    public bool Advance()
    {
        if (frameNumber >= totalFrameCount) return false;

        frameNumber++;
        foreach (CameraSetup cameraSetup in cameras)
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
        float totalError = Calculate3DPosesAndTotalError();

        int iterationCount = 0;
        while (iterationCount < 10000)
        {
            CameraSetup highestErrorCam = null;
            float highestError = 0;
            foreach (CameraSetup cameraSetup in cameras)
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
        foreach (CameraSetup cameraSetup in cameras)
        {
            cameraSetup.Project(frameNumber);
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
            foreach (CameraSetup cameraSetup in cameras)
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
        foreach (CameraSetup cameraSetup in cameras)
        {
            totalError += cameraSetup.Error(merged3DPoseLeadPerFrame[frameNumber],
                merged3DPoseFollowPerFrame[frameNumber], frameNumber);
        }

        return totalError;
    }

    static int FrameWithHighestConfidence(
        IReadOnlyList<List<List<Vector3>>> leadByCamera,
        IReadOnlyList<List<List<Vector3>>> followByCamera)
    {
        int frameWithHighestConfidence = 0;
        float highestConfidence = 0f;

        int highestValidIndex = leadByCamera.Select(t => t.Count).Min();

        for (int i = 0; i < leadByCamera.Count; i++)
        {
            for (int j = 0; j < highestValidIndex; j++)
            {
                float confidence = Confidence(leadByCamera[i][j], followByCamera[i][j]);
                if (confidence > highestConfidence)
                {
                    highestConfidence = confidence;
                    frameWithHighestConfidence = j;
                }
            }
        }

        return frameWithHighestConfidence;
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
            merged3DPoseLeadPerFrame[frameNumber][(int)Halpe.LAnkle].Y,
            merged3DPoseLeadPerFrame[frameNumber][(int)Halpe.RAnkle].Y);
        return lowestLeadAnkle > 0.1f; // 10cm above ground
    }

    public bool AnyPointsHigherThan2pt5Meters()
    {
        return merged3DPoseLeadPerFrame[frameNumber].Any(vec => vec.Y > 2.5) ||
               merged3DPoseFollowPerFrame[frameNumber].Any(vec => vec.Y > 2.5);
    }

    public int GetFrameNumber() => frameNumber;
    public int GetTotalFrameCount() => totalFrameCount;
}