using System.Numerics;
using Newtonsoft.Json;

namespace dancer_pose_alignment;

public class CameraPoseSolver
{
    readonly List<List<Vector3>> merged3DPoseLeadPerFrame = [];
    readonly List<List<Vector3>> merged3DPoseFollowPerFrame = [];

    readonly int poseCount = 26; // Halpe
    readonly List<CameraSetup> cameras = [];

    public void LoadPoses(string directoryPath)
    {
        List<Vector3> cameraPositions = [];
        List<float> cameraFocalLengths = [];
        List<Vector2> cameraSizes = [];

        Dictionary<int, List<List<Vector3>>> allLeadPosesByCamera = [];
        Dictionary<int, List<List<Vector3>>> allFollowPosesByCamera = [];
        Dictionary<int, List<List<Vector3>>> allMirrorLeadPosesByCamera = [];
        Dictionary<int, List<List<Vector3>>> allMirrorFollowPosesByCamera = [];
        int totalFrameCount = 0;

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
                else if (fileName.StartsWith("camera-focal-lengths"))
                {
                    string jsonContent = File.ReadAllText(file);
                    cameraFocalLengths = JsonConvert.DeserializeObject<List<float>>(jsonContent);
                }
                else if (fileName.StartsWith("cameraSizes"))
                {
                    string jsonContent = File.ReadAllText(file);
                    cameraSizes = JsonConvert.DeserializeObject<List<Vector2>>(jsonContent);
                }
                else
                {
                    List<List<Tuple<int, bool>>> cameraIndicesByFrame =
                        JsonConvert.DeserializeObject<List<List<Tuple<int, bool>>>>(File.ReadAllText(file));
                }
            }
        }

        // seed
        for (int i = 0; i < totalFrameCount; i++)
        {
            merged3DPoseLeadPerFrame.Add([]);
            merged3DPoseFollowPerFrame.Add([]);
        }

        CreateAndPlaceCameras(cameraSizes, cameraPositions, cameraFocalLengths);
        AddPosesToCamera(
            allLeadPosesByCamera,
            allFollowPosesByCamera,
            allMirrorLeadPosesByCamera,
            allMirrorFollowPosesByCamera);
    }

    public List<List<List<Vector3>>> AllPosesAtFramePerCamera(int frameNumber)
    {
        return cameras.Select(camera => camera.PosesPerDancerAtFrame(frameNumber).ToList()).ToList();
    }

    public List<List<Vector2>> ReverseProjectionOfLeadPosePerCamera(int frameNumber)
    {
        return cameras.Select(camera => merged3DPoseLeadPerFrame[frameNumber]
                .Select(x => camera.ReverseProjectPoint(x, frameNumber)).ToList())
            .ToList();
    }
    
    public List<List<Vector2>> ReverseProjectionOfFollowPosePerCamera(int frameNumber)
    {
        return cameras.Select(camera => merged3DPoseFollowPerFrame[frameNumber]
                .Select(x => camera.ReverseProjectPoint(x, frameNumber)).ToList())
            .ToList();
    }
    
    public List<Vector2> ReverseProjectOriginsPerCamera(int frameNumber)
    {
        return cameras.Select(camera => camera.ReverseProjectPoint(Vector3.Zero, frameNumber)).ToList();
    }

    void CreateAndPlaceCameras(
        IReadOnlyList<Vector2> imageSizes,
        IReadOnlyList<Vector3> seedCameraPositions,
        IReadOnlyList<float> seedCameraFocalLengths)
    {
        const int testingFrameNumber = 0;
        for (int i = 0; i < imageSizes.Count; i++)
        {
            CameraSetup camera = new()
            {
                Size = imageSizes[i],
                FocalLength = seedCameraFocalLengths[i]
            };

            if (seedCameraPositions.Count != 0)
            {
                camera.PositionsPerFrame.Add(seedCameraPositions[i]);
                // TODO confirm this rotation works
                Quaternion centerLook = Transform.LookAt(
                    new Vector3(0, 1.5f, 0),
                    Quaternion.Identity,
                    camera.PositionsPerFrame[testingFrameNumber]);
                camera.RotationsPerFrame.Add(centerLook);
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

                camera.RotationsPerFrame.Add(Transform.LookAt(
                    new Vector3(0, startingHeight, 0),
                    Quaternion.Identity,
                    camera.PositionsPerFrame[testingFrameNumber]));
            }

            cameras.Add(camera);
        }
    }

    void AddPosesToCamera(
        IReadOnlyDictionary<int, List<List<Vector3>>> allLeadPosesByCamera,
        IReadOnlyDictionary<int, List<List<Vector3>>> allFollowPosesByCamera,
        IReadOnlyDictionary<int, List<List<Vector3>>> allMirrorLeadPosesByCamera,
        IReadOnlyDictionary<int, List<List<Vector3>>> allMirrorFollowPosesByCamera)
    {
        int count = 0;
        foreach (CameraSetup camera in cameras)
        {
            camera.AddPosesAndRecenterAndScaleToCamera(
                allLeadPosesByCamera[count],
                allFollowPosesByCamera[count],
                allMirrorLeadPosesByCamera[count],
                allMirrorFollowPosesByCamera[count]);
            count++;
        }
    }

    public void IterationLoop()
    {
        const int testingFrameNumber = 0;
        float totalError = Calculate3DPosesAndTotalError(testingFrameNumber);

        int iterationCount = 0;
        while (totalError > 3f && iterationCount < 1000000)
        {
            totalError = Iterate(
                testingFrameNumber,
                totalError);
            iterationCount++;
        }

        Console.WriteLine(totalError);
    }

    public void SaveData()
    {
        string jsonCameras = JsonConvert.SerializeObject(cameras, Formatting.Indented);
        File.WriteAllText(Path.Combine(@"C:\Users\john\Desktop", "cameras.json"), jsonCameras);
        Console.WriteLine("wrote cameras.json");

        string jsonMerged3DPose = JsonConvert.SerializeObject(merged3DPoseLeadPerFrame, Formatting.Indented);
        File.WriteAllText(Path.Combine(@"C:\Users\john\Desktop", "merged3DPoseLead.json"), jsonMerged3DPose);
        Console.WriteLine("wrote merged3DPoseLead.json");

        string jsonMerged3DPoseFollow = JsonConvert.SerializeObject(merged3DPoseFollowPerFrame, Formatting.Indented);
        File.WriteAllText(Path.Combine(@"C:\Users\john\Desktop", "merged3DPoseFollow.json"), jsonMerged3DPoseFollow);
        Console.WriteLine("wrote merged3DPoseFollow.json");
    }

    float Iterate(int frameNumber, float lastIterationError)
    {
        float previousError = lastIterationError;
        const float errorMin = .02f;
        while (true)
        {
            float yawError = IterateYaw(frameNumber, .05f);
            if (Math.Abs(yawError - previousError) < errorMin) break;
            previousError = yawError;
            Console.WriteLine("Yawing: " + previousError);
        }

        while (true)
        {
            float pitchError = IteratePitch(frameNumber, .05f);
            if (Math.Abs(pitchError - previousError) < errorMin) break;
            previousError = pitchError;
            Console.WriteLine("Pitching: " + previousError);
        }

        while (true)
        {
            float focalError = IterateFocal(frameNumber, .005f);
            if (Math.Abs(focalError - previousError) < errorMin * .1f) break;
            previousError = focalError;
            Console.WriteLine("Focusing: " + previousError);
        }

        while (true)
        {
            float rollError = IterateRoll(frameNumber, .01f);
            if (Math.Abs(rollError - previousError) < errorMin) break;
            previousError = rollError;
            Console.WriteLine("Rolling: " + previousError);
        }

        while (true)
        {
            float xPositionError = IterateXPosition(frameNumber, .1f);
            if (Math.Abs(xPositionError - previousError) < errorMin) break;
            previousError = xPositionError;
            Console.WriteLine("Moving in X: " + previousError);
        }

        while (true)
        {
            float zPositionError = IterateZPosition(frameNumber, .1f);
            if (Math.Abs(zPositionError - previousError) < errorMin) break;
            previousError = zPositionError;
            Console.WriteLine("Moving in Z: " + previousError);
        }

        if (previousError < 3f)
        {
            return previousError;
        }

        while (true)
        {
            float heightError = IterateYPosition(frameNumber, .1f);
            if (Math.Abs(heightError - previousError) < errorMin) break;
            previousError = heightError;
            Console.WriteLine("Moving in Y: " + previousError);
        }

        return previousError;
    }

    float IterateYaw(int frameNumber, float yawStepSize)
    {
        foreach (CameraSetup camera in cameras)
        {
            float currentError = Calculate3DPosesAndTotalError(frameNumber);

            // yaw camera left and right
            Quaternion originalRotation = camera.RotationsPerFrame[frameNumber];

            Quaternion leftYawRotation = originalRotation *
                                         Quaternion.CreateFromAxisAngle(new Vector3(0, 1, 0), yawStepSize);
            Quaternion rightYawRotation = originalRotation *
                                          Quaternion.CreateFromAxisAngle(new Vector3(0, 1, 0), -yawStepSize);

            camera.RotationsPerFrame[frameNumber] = leftYawRotation;
            float leftYawError = Calculate3DPosesAndTotalError(frameNumber);

            camera.RotationsPerFrame[frameNumber] = rightYawRotation;
            float rightYawError = Calculate3DPosesAndTotalError(frameNumber);

            if (leftYawError < rightYawError && leftYawError < currentError)
            {
                camera.RotationsPerFrame[frameNumber] = leftYawRotation;
            }
            else if (rightYawError < leftYawError && rightYawError < currentError)
            {
                camera.RotationsPerFrame[frameNumber] = rightYawRotation;
            }
            else
            {
                camera.RotationsPerFrame[frameNumber] = originalRotation;
            }
        }

        return Calculate3DPosesAndTotalError(frameNumber);
    }

    float IteratePitch(int frameNumber, float pitchStepSize)
    {
        foreach (CameraSetup camera in cameras)
        {
            float currentError = Calculate3DPosesAndTotalError(frameNumber);

            // pitch camera up and down
            Quaternion originalRotation = camera.RotationsPerFrame[frameNumber];
            Quaternion upPitchRotation = originalRotation *
                                         Quaternion.CreateFromAxisAngle(new Vector3(1, 0, 0), -pitchStepSize);

            // check if the angle between the up pitch forward vector and the ground forward vector is greater than 15 degrees
            Vector3 upPitchForward = Vector3.Normalize(Vector3.Transform(new Vector3(0, 0, 1), upPitchRotation));

            float angleBetween = MathF.Acos(Vector3.Dot(upPitchForward, camera.Forward(frameNumber)));
            if (angleBetween > MathF.PI / 12)
            {
                Console.WriteLine("pitch angle too high" + angleBetween);
                camera.RotationsPerFrame[frameNumber] = originalRotation;
            }

            Quaternion downPitchRotation = originalRotation *
                                           Quaternion.CreateFromAxisAngle(new Vector3(0, 0, 1), pitchStepSize);

            // check if the angle between the down pitch forward vector and the ground forward vector is greater than 15 degrees
            Vector3 downPitchForward = Vector3.Normalize(Vector3.Transform(new Vector3(0, 0, 1), downPitchRotation));
            angleBetween = MathF.Acos(Vector3.Dot(downPitchForward, camera.Forward(frameNumber)));
            if (angleBetween > MathF.PI / 12)
            {
                Console.WriteLine("pitch angle too low" + angleBetween);
                camera.RotationsPerFrame[frameNumber] = originalRotation;
            }

            camera.RotationsPerFrame[frameNumber] = upPitchRotation;
            float upPitchError = Calculate3DPosesAndTotalError(frameNumber);
            if (AnyPointsHigherThan2pt5Meters(frameNumber) || LowestLeadAnkleIsMoreThan10CMAboveZero(frameNumber))
            {
                upPitchError = float.MaxValue;
            }

            camera.RotationsPerFrame[frameNumber] = downPitchRotation;
            float downPitchError = Calculate3DPosesAndTotalError(frameNumber);
            if (AnyPointsBelowGround(frameNumber))
            {
                downPitchError = float.MaxValue;
            }

            if (upPitchError < downPitchError && upPitchError < currentError)
            {
                camera.RotationsPerFrame[frameNumber] = upPitchRotation;
            }
            else if (downPitchError < upPitchError && downPitchError < currentError)
            {
                camera.RotationsPerFrame[frameNumber] = downPitchRotation;
            }
            else
            {
                camera.RotationsPerFrame[frameNumber] = originalRotation;
            }
        }

        return Calculate3DPosesAndTotalError(frameNumber);
    }

    float IterateFocal(int frameNumber, float focalStepSize)
    {
        foreach (CameraSetup camera in cameras)
        {
            float thisError = Calculate3DPosesAndTotalError(frameNumber);

            // adjust focal length
            float originalFocalLength = camera.FocalLength;

            camera.FocalLength = originalFocalLength + focalStepSize;
            if (camera.FocalLength > .6f)
            {
                Console.WriteLine($"focal length max: {cameras.IndexOf(camera)} + {camera.FocalLength}");
                camera.FocalLength = originalFocalLength;
            }

            float zoomInFocalError = Calculate3DPosesAndTotalError(frameNumber);

            camera.FocalLength = originalFocalLength - focalStepSize;
            if (camera.FocalLength < .001f)
            {
                Console.WriteLine($"focal length min: {cameras.IndexOf(camera)} + {camera.FocalLength}");
                camera.FocalLength = .001f;
            }

            float zoomOutFocalError = Calculate3DPosesAndTotalError(frameNumber);
            if (AnyPointsBelowGround(frameNumber))
            {
                zoomOutFocalError = float.MaxValue;
            }

            if (zoomInFocalError < zoomOutFocalError && zoomInFocalError < thisError)
            {
                camera.FocalLength = originalFocalLength + focalStepSize;
            }
            else if (zoomOutFocalError < zoomInFocalError && zoomOutFocalError < thisError)
            {
                camera.FocalLength = originalFocalLength - focalStepSize;
            }
            else
            {
                camera.FocalLength = originalFocalLength;
            }
        }

        return Calculate3DPosesAndTotalError(frameNumber);
    }

    float IterateRoll(int frameNumber, float rollStepSize)
    {
        foreach (CameraSetup camera in cameras)
        {
            float currentError = Calculate3DPosesAndTotalError(frameNumber);
            // roll camera left and right
            Quaternion originalRotation = camera.RotationsPerFrame[frameNumber];

            Quaternion leftRollRotation = originalRotation *
                                          Quaternion.CreateFromAxisAngle(new Vector3(0, 0, 1), rollStepSize);

            // check if the up vector has rolled too far to the left relative to the ground up vector
            Vector3 leftRollUp = Vector3.Normalize(Vector3.Transform(new Vector3(0, 1, 0), leftRollRotation));

            float angleBetween = MathF.Acos(Vector3.Dot(leftRollUp, camera.Up(frameNumber)));
            if (angleBetween > MathF.PI / 12)
            {
                Console.WriteLine("roll angle too far to the left" + angleBetween);
                camera.RotationsPerFrame[frameNumber] = originalRotation;
            }

            Quaternion rightRollRotation = originalRotation *
                                           Quaternion.CreateFromAxisAngle(new Vector3(0, 0, 1), -rollStepSize);

            // check if the up vector has rolled too far to the right relative to the ground up vector
            Vector3 rightRollUp = Vector3.Transform(new Vector3(0, 1, 0), rightRollRotation);
            angleBetween = MathF.Acos(Vector3.Dot(rightRollUp, camera.Up(frameNumber)));
            if (angleBetween > MathF.PI / 12)
            {
                Console.WriteLine("roll angle too far to the right" + angleBetween);
                camera.RotationsPerFrame[frameNumber] = originalRotation;
            }

            camera.RotationsPerFrame[frameNumber] = leftRollRotation;
            float leftRollError = Calculate3DPosesAndTotalError(frameNumber);

            camera.RotationsPerFrame[frameNumber] = rightRollRotation;
            float rightRollError = Calculate3DPosesAndTotalError(frameNumber);

            if (leftRollError < rightRollError && leftRollError < currentError)
            {
                camera.RotationsPerFrame[frameNumber] = leftRollRotation;
            }
            else if (rightRollError < leftRollError && rightRollError < currentError)
            {
                camera.RotationsPerFrame[frameNumber] = rightRollRotation;
            }
            else
            {
                camera.RotationsPerFrame[frameNumber] = originalRotation;
            }
        }

        return Calculate3DPosesAndTotalError(frameNumber);
    }

    float IterateXPosition(int frameNumber, float stepSize)
    {
        foreach (CameraSetup camera in cameras)
        {
            float currentError = Calculate3DPosesAndTotalError(frameNumber);
            Vector3 cameraOriginalPosition = camera.PositionsPerFrame[frameNumber];
            Quaternion cameraOriginalRotation = camera.RotationsPerFrame[frameNumber];
            float cameraOriginalFocalLength = camera.FocalLength;

            // move camera left
            Vector3 leftPosition = cameraOriginalPosition with { X = cameraOriginalPosition.X - stepSize };
            camera.PositionsPerFrame[frameNumber] = leftPosition;

            float translateLeftError = Calculate3DPosesAndTotalError(frameNumber);
            float lastLoopError = translateLeftError;
            float yawLeftCorrectionError = IterateYaw(frameNumber, .05f);
            while (yawLeftCorrectionError < lastLoopError && Math.Abs(yawLeftCorrectionError - lastLoopError) > .02f)
            {
                // recenter to make a fair comparison
                lastLoopError = yawLeftCorrectionError;
                yawLeftCorrectionError = IterateYaw(frameNumber, .05f);
            }

            lastLoopError = translateLeftError;
            float iterateLeftFocalError = IterateFocal(frameNumber, .005f);
            while (iterateLeftFocalError < lastLoopError && Math.Abs(iterateLeftFocalError - lastLoopError) > .02f)
            {
                // recenter to make a fair comparison
                lastLoopError = iterateLeftFocalError;
                iterateLeftFocalError = IterateFocal(frameNumber, .005f);
            }

            translateLeftError = lastLoopError;
            Quaternion leftYawRotation = camera.RotationsPerFrame[frameNumber];
            float leftFocalLength = camera.FocalLength;

            //reset
            camera.RotationsPerFrame[frameNumber] = cameraOriginalRotation;
            camera.FocalLength = cameraOriginalFocalLength;

            // move camera right
            Vector3 rightPosition = cameraOriginalPosition with { X = cameraOriginalPosition.X + stepSize };
            camera.PositionsPerFrame[frameNumber] = rightPosition;

            float translateRightError = Calculate3DPosesAndTotalError(frameNumber);
            lastLoopError = translateRightError;
            float yawRightCorrectionError = IterateYaw(frameNumber, .05f);
            while (yawRightCorrectionError < lastLoopError && Math.Abs(yawRightCorrectionError - lastLoopError) > .02f)
            {
                // recenter to make a fair comparison
                lastLoopError = yawRightCorrectionError;
                yawRightCorrectionError = IterateYaw(frameNumber, .05f);
            }

            lastLoopError = translateRightError;
            float iterateRightFocalError = IterateFocal(frameNumber, .005f);
            while (iterateRightFocalError < lastLoopError && Math.Abs(iterateRightFocalError - lastLoopError) > .02f)
            {
                // recenter to make a fair comparison
                lastLoopError = iterateRightFocalError;
                iterateRightFocalError = IterateFocal(frameNumber, .005f);
            }

            translateRightError = lastLoopError;

            if (translateLeftError < translateRightError && translateLeftError < currentError)
            {
                // reset to left correction
                camera.PositionsPerFrame[frameNumber] = leftPosition;
                camera.RotationsPerFrame[frameNumber] = leftYawRotation;
                camera.FocalLength = leftFocalLength;
            }
            else if (translateRightError < translateLeftError && translateRightError < currentError)
            {
                // leave rotation and yaw
                camera.PositionsPerFrame[frameNumber] = rightPosition;
            }
            else
            {
                // reset
                camera.PositionsPerFrame[frameNumber] = cameraOriginalPosition;
                camera.RotationsPerFrame[frameNumber] = cameraOriginalRotation;
                camera.FocalLength = cameraOriginalFocalLength;
            }
        }

        return Calculate3DPosesAndTotalError(frameNumber);
    }

    float IterateZPosition(int frameNumber, float stepSize)
    {
        foreach (CameraSetup camera in cameras)
        {
            float currentError = Calculate3DPosesAndTotalError(frameNumber);
            Vector3 cameraOriginalPosition = camera.PositionsPerFrame[frameNumber];
            Quaternion cameraOriginalRotation = camera.RotationsPerFrame[frameNumber];
            float cameraOriginalFocalLength = camera.FocalLength;

            // move camera back
            Vector3 backPosition = cameraOriginalPosition with { Z = cameraOriginalPosition.Z - stepSize };
            camera.PositionsPerFrame[frameNumber] = backPosition;

            float translateBackError = Calculate3DPosesAndTotalError(frameNumber);
            float lastLoopError = translateBackError;
            float yawBackCorrectionError = IterateYaw(frameNumber, .05f);
            while (yawBackCorrectionError < lastLoopError && Math.Abs(yawBackCorrectionError - lastLoopError) > .02f)
            {
                // recenter to make a fair comparison
                lastLoopError = yawBackCorrectionError;
                yawBackCorrectionError = IterateYaw(frameNumber, .05f);
            }

            lastLoopError = translateBackError;
            float iterateBackFocalError = IterateFocal(frameNumber, .005f);
            while (iterateBackFocalError < lastLoopError && Math.Abs(iterateBackFocalError - lastLoopError) > .02f)
            {
                // recenter to make a fair comparison
                lastLoopError = iterateBackFocalError;
                iterateBackFocalError = IterateFocal(frameNumber, .005f);
            }

            Quaternion backYawRotation = camera.RotationsPerFrame[frameNumber];
            float backFocalLength = camera.FocalLength;

            // reset
            camera.RotationsPerFrame[frameNumber] = cameraOriginalRotation;
            camera.FocalLength = cameraOriginalFocalLength;
            translateBackError = lastLoopError;

            // move camera forward
            Vector3 forwardPosition = cameraOriginalPosition with { Z = cameraOriginalPosition.Z + stepSize };
            camera.PositionsPerFrame[frameNumber] = forwardPosition;

            float translateForwardError = Calculate3DPosesAndTotalError(frameNumber);
            lastLoopError = translateForwardError;
            float yawForwardCorrectionError = IterateYaw(frameNumber, .05f);
            while (yawForwardCorrectionError < lastLoopError &&
                   Math.Abs(yawForwardCorrectionError - lastLoopError) > .02f)
            {
                // recenter to make a fair comparison
                lastLoopError = yawForwardCorrectionError;
                yawForwardCorrectionError = IterateYaw(frameNumber, .05f);
            }

            lastLoopError = translateForwardError;
            float iterateForwardFocalError = IterateFocal(frameNumber, .005f);
            while (iterateForwardFocalError < lastLoopError &&
                   Math.Abs(iterateForwardFocalError - lastLoopError) > .02f)
            {
                // recenter to make a fair comparison
                lastLoopError = iterateForwardFocalError;
                iterateForwardFocalError = IterateFocal(frameNumber, .005f);
            }

            translateForwardError = lastLoopError;

            if (translateBackError < translateForwardError && translateBackError < currentError)
            {
                // set to back
                camera.PositionsPerFrame[frameNumber] = backPosition;
                camera.RotationsPerFrame[frameNumber] = backYawRotation;
                camera.FocalLength = backFocalLength;
            }
            else if (translateForwardError < translateBackError && translateForwardError < currentError)
            {
                // leave
                camera.PositionsPerFrame[frameNumber] = forwardPosition;
            }
            else
            {
                // reset
                camera.PositionsPerFrame[frameNumber] = cameraOriginalPosition;
                camera.RotationsPerFrame[frameNumber] = cameraOriginalRotation;
                camera.FocalLength = cameraOriginalFocalLength;
            }
        }

        return Calculate3DPosesAndTotalError(frameNumber);
    }

    float IterateYPosition(int frameNumber, float stepSize)
    {
        foreach (CameraSetup camera in cameras)
        {
            float currentError = Calculate3DPosesAndTotalError(frameNumber);
            Vector3 cameraOriginalPosition = camera.PositionsPerFrame[frameNumber];
            Quaternion cameraOriginalRotation = camera.RotationsPerFrame[frameNumber];

            Vector3 upPosition = cameraOriginalPosition;
            if (cameraOriginalPosition.Y + stepSize < 2.3f)
            {
                upPosition = cameraOriginalPosition with { Y = cameraOriginalPosition.Y + stepSize };
            }

            camera.PositionsPerFrame[frameNumber] = upPosition;

            // recenter pitch to make a fair comparison
            float moveUpError = Calculate3DPosesAndTotalError(frameNumber);
            float lastLoopError = moveUpError;
            float pitchUpCorrectionError = IteratePitch(frameNumber, .05f);
            while (pitchUpCorrectionError < lastLoopError && Math.Abs(pitchUpCorrectionError - lastLoopError) > .02f)
            {
                // recenter to make a fair comparison
                lastLoopError = pitchUpCorrectionError;
                pitchUpCorrectionError = IteratePitch(frameNumber, .05f);
            }

            // reset
            lastLoopError = moveUpError;
            Quaternion upPitchRotation = camera.RotationsPerFrame[frameNumber];
            camera.RotationsPerFrame[frameNumber] = cameraOriginalRotation;

            Vector3 downPosition = cameraOriginalPosition;
            if (cameraOriginalPosition.Y - stepSize > 0)
            {
                downPosition = cameraOriginalPosition with { Y = cameraOriginalPosition.Y - stepSize };
            }

            camera.PositionsPerFrame[frameNumber] = downPosition;

            // recenter pitch to make a fair comparison
            float pitchDownCorrectionError = IteratePitch(frameNumber, .05f);
            while (pitchDownCorrectionError < lastLoopError &&
                   Math.Abs(pitchDownCorrectionError - lastLoopError) > .02f)
            {
                // recenter to make a fair comparison
                lastLoopError = pitchDownCorrectionError;
                pitchDownCorrectionError = IteratePitch(frameNumber, .05f);
            }

            if (moveUpError < lastLoopError && moveUpError < currentError)
            {
                // set to up
                camera.PositionsPerFrame[frameNumber] = upPosition;
                camera.RotationsPerFrame[frameNumber] = upPitchRotation;
            }
            else if (lastLoopError < moveUpError && lastLoopError < currentError)
            {
                // set to down
                camera.PositionsPerFrame[frameNumber] = downPosition;
            }
            else
            {
                // reset
                camera.PositionsPerFrame[frameNumber] = cameraOriginalPosition;
                camera.RotationsPerFrame[frameNumber] = cameraOriginalRotation;
            }
        }

        return Calculate3DPosesAndTotalError(frameNumber);
    }

    public float Calculate3DPosesAndTotalError(int frameNumber)
    {
        foreach (CameraSetup cameraSetup in cameras)
        {
            cameraSetup.Project(frameNumber);
        }

        merged3DPoseLeadPerFrame[frameNumber].Clear();
        merged3DPoseFollowPerFrame[frameNumber].Clear();
        for (int i = 0; i < poseCount; i++)
        {
            List<Ray> leadRays = [];
            List<Ray> followRays = [];
            List<float> leadConfidences = [];
            List<float> followConfidences = [];
            foreach (CameraSetup cameraSetup in cameras)
            {
                if (cameraSetup.HasPoseAtFrame(frameNumber, true))
                {
                    leadRays.Add(cameraSetup.PoseRay(frameNumber, i, true));
                    leadConfidences.Add(cameraSetup.JointConfidence(frameNumber, i, true));
                }

                if (cameraSetup.HasPoseAtFrame(frameNumber, false))
                {
                    followRays.Add(cameraSetup.PoseRay(frameNumber, i, false));
                    followConfidences.Add(cameraSetup.JointConfidence(frameNumber, i, false));
                }
            }

            Vector3 leadJointMidpoint = RayMidpointFinder.FindMinimumMidpoint(leadRays, leadConfidences);
            merged3DPoseLeadPerFrame[frameNumber].Add(leadJointMidpoint);
            Vector3 followJointMidpoint = RayMidpointFinder.FindMinimumMidpoint(followRays, followConfidences);
            merged3DPoseFollowPerFrame[frameNumber].Add(followJointMidpoint);
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

    bool AnyPointsBelowGround(int frameNumber) => merged3DPoseLeadPerFrame[frameNumber].Any(vec => vec.Y < 0) ||
                                                  merged3DPoseFollowPerFrame[frameNumber].Any(vec => vec.Y < 0);

    bool LowestLeadAnkleIsMoreThan10CMAboveZero(int frameNumber)
    {
        float lowestLeadAnkle = Math.Min(
            merged3DPoseLeadPerFrame[frameNumber][(int)Halpe.LAnkle].Y,
            merged3DPoseLeadPerFrame[frameNumber][(int)Halpe.RAnkle].Y);
        return lowestLeadAnkle > 0.1f;
    }

    bool AnyPointsHigherThan2pt5Meters(int frameNumber)
    {
        return merged3DPoseLeadPerFrame[frameNumber].Any(vec => vec.Y > 2.5) ||
               merged3DPoseFollowPerFrame[frameNumber].Any(vec => vec.Y > 2.5);
    }
}