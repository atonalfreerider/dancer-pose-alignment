using System.Numerics;
using Newtonsoft.Json;

namespace dancer_pose_alignment;

public class CameraPoseSolver
{
    readonly List<Vector3> merged3DPoseLead = [];
    readonly List<Vector3> merged3DPoseFollow = [];

    List<Vector2> imageSizes;

    readonly List<List<List<Vector3>>> leadByCamera = [];
    readonly List<List<List<Vector3>>> followByCamera = [];

    readonly int poseCount = 26;

    readonly List<CameraSetup> cameras = [];

    public void LoadPoses(string inputPath, string jsonCameraSizes)
    {
        imageSizes = JsonConvert.DeserializeObject<List<Vector2>>(File.ReadAllText(jsonCameraSizes));

        foreach (string jsonPath in Directory.EnumerateFiles(inputPath, "*.json"))
        {
            List<List<Vector3>> finalIndexListLeadAndFollow =
                JsonConvert.DeserializeObject<List<List<Vector3>>>(File.ReadAllText(jsonPath));

            if (jsonPath.Contains("lead"))
            {
                leadByCamera.Add(finalIndexListLeadAndFollow);
            }
            else
            {
                followByCamera.Add(finalIndexListLeadAndFollow);
            }
        }

        PlaceCamerasInCircle();
        AddPosesToCamera();
    }

    void PlaceCamerasInCircle()
    {
        const int testingFrameNumber = 0;
        for (int i = 0; i < followByCamera.Count; i++)
        {
            // position cameras in a circle
            CameraSetup camera = new()
            {
                Size = imageSizes[i]
            };

            float angle = -(float)i / followByCamera.Count * 2 * MathF.PI; // ccw
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

            cameras.Add(camera);
        }
    }

    void AddPosesToCamera()
    {
        int frameWithHighestConfidence = FrameWithHighestConfidence(leadByCamera, followByCamera);
        int count = 0;
        foreach (CameraSetup camera in cameras)
        {
            List<Vector3> leadAtCamera = leadByCamera[count][frameWithHighestConfidence];
            List<Vector3> followAtCamera = followByCamera[count][frameWithHighestConfidence];

            camera.AddPosesAndRecenterAndScaleToCamera(leadAtCamera, followAtCamera);
            count++;
        }
    }

    public void IterationLoop()
    {
        const int testingFrameNumber = 0;
        float totalError = Calculate3DPosesAndTotalError(testingFrameNumber);

        while (AnyPointsBelowGround)
        {
            foreach (CameraSetup camera in cameras)
            {
                // pitch the camera up by .01 radians
                Quaternion upPitchRotation = camera.RotationsPerFrame[testingFrameNumber] *
                                             Quaternion.CreateFromAxisAngle(new Vector3(1, 0, 0),
                                                 -.01f);

                // check if the angle between the up pitch forward vector and the ground forward vector is too large
                Vector3 upPitchForward = Vector3.Normalize(
                    Vector3.Transform(new Vector3(0, 0, 1), upPitchRotation));
                Vector3 cameraForwardParallelToGround = Vector3.Normalize(new Vector3(
                    -camera.PositionsPerFrame[testingFrameNumber].X,
                    0,
                    -camera.PositionsPerFrame[testingFrameNumber].Z));

                float angleBetween = MathF.Acos(Vector3.Dot(upPitchForward, cameraForwardParallelToGround));
                Console.WriteLine(angleBetween);
                if (angleBetween > MathF.PI / 6)
                {
                    Console.WriteLine("pitch maxed out");
                    upPitchRotation = camera.RotationsPerFrame[testingFrameNumber];
                }

                camera.RotationsPerFrame[testingFrameNumber] = upPitchRotation;

                camera.FocalLength += .005f;
                if (camera.FocalLength > .35f)
                {
                    Console.WriteLine("focus maxed out");
                    camera.FocalLength = 0.35f;
                }
            }

            Console.WriteLine("Pitching and extending focus");
            totalError = Calculate3DPosesAndTotalError(testingFrameNumber);
        }

        int iterationCount = 0;
        while (totalError > .1f && iterationCount < 1000000)
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

        string jsonMerged3DPose = JsonConvert.SerializeObject(merged3DPoseLead, Formatting.Indented);
        File.WriteAllText(Path.Combine(@"C:\Users\john\Desktop", "merged3DPoseLead.json"), jsonMerged3DPose);
        Console.WriteLine("wrote merged3DPoseLead.json");

        string jsonMerged3DPoseFollow = JsonConvert.SerializeObject(merged3DPoseFollow, Formatting.Indented);
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
                Console.WriteLine("pitch angle too high" + angleBetween);
                camera.RotationsPerFrame[frameNumber] = originalRotation;
            }

            camera.RotationsPerFrame[frameNumber] = upPitchRotation;
            float upPitchError = Calculate3DPosesAndTotalError(frameNumber);

            camera.RotationsPerFrame[frameNumber] = downPitchRotation;
            float downPitchError = Calculate3DPosesAndTotalError(frameNumber);
            if (AnyPointsBelowGround)
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
            if (camera.FocalLength > .35f)
            {
                Console.WriteLine("focal length max" + camera.FocalLength);
                camera.FocalLength = originalFocalLength;
            }

            float upFocalError = Calculate3DPosesAndTotalError(frameNumber);

            camera.FocalLength = originalFocalLength - focalStepSize;
            if (camera.FocalLength < .001f)
            {
                Console.WriteLine("focal length min" + camera.FocalLength);
                camera.FocalLength = .001f;
            }

            float downFocalError = Calculate3DPosesAndTotalError(frameNumber);
            if (AnyPointsBelowGround)
            {
                downFocalError = float.MaxValue;
            }

            if (upFocalError < downFocalError && upFocalError < thisError)
            {
                camera.FocalLength = originalFocalLength + focalStepSize;
            }
            else if (downFocalError < upFocalError && downFocalError < thisError)
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
            while (yawForwardCorrectionError < lastLoopError && Math.Abs(yawForwardCorrectionError - lastLoopError) > .02f)
            {
                // recenter to make a fair comparison
                lastLoopError = yawForwardCorrectionError;
                yawForwardCorrectionError = IterateYaw(frameNumber, .05f);
            }

            lastLoopError = translateForwardError;
            float iterateForwardFocalError = IterateFocal(frameNumber, .005f);
            while (iterateForwardFocalError < lastLoopError && Math.Abs(iterateForwardFocalError - lastLoopError) > .02f)
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
            while (pitchDownCorrectionError < lastLoopError && Math.Abs(pitchDownCorrectionError - lastLoopError) > .02f)
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

    float Calculate3DPosesAndTotalError(int frameNumber)
    {
        foreach (CameraSetup cameraSetup in cameras)
        {
            cameraSetup.Project(frameNumber);
        }

        merged3DPoseLead.Clear();
        merged3DPoseFollow.Clear();
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
            merged3DPoseLead.Add(leadJointMidpoint);
            Vector3 followJointMidpoint = RayMidpointFinder.FindMinimumMidpoint(followRays, followConfidences);
            merged3DPoseFollow.Add(followJointMidpoint);
        }

        float totalError = 0;
        foreach (CameraSetup cameraSetup in cameras)
        {
            totalError += cameraSetup.Error(merged3DPoseLead, merged3DPoseFollow, frameNumber);
        }

        return totalError;
    }

    static int FrameWithHighestConfidence(
        IReadOnlyList<List<List<Vector3>>> leadByCamera,
        IReadOnlyList<List<List<Vector3>>> followByCamera)
    {
        int frameWithHighestConfidence = 0;
        float highestConfidence = 0f;
        for (int i = 0; i < leadByCamera.Count; i++)
        {
            for (int j = 0; j < 896; j++)
            {
                float confidence = Confidence(followByCamera[i][j], leadByCamera[i][j]);
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

    bool AnyPointsBelowGround => merged3DPoseLead.Any(vec => vec.Y < 0) || merged3DPoseFollow.Any(vec => vec.Y < 0);
}