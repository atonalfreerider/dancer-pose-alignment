using System.Numerics;
using Newtonsoft.Json;

namespace dancer_pose_alignment;

public class CameraPoseSolver
{
    readonly List<Vector3> merged3DPoseLead = [];
    readonly List<Vector3> merged3DPoseFollow = [];

    public void LoadPoses(string inputPath, string jsonCameraSizes)
    {
        List<Vector2> imageSizes = JsonConvert.DeserializeObject<List<Vector2>>(File.ReadAllText(jsonCameraSizes));

        List<List<List<Vector3>>> leadByCamera = [];
        List<List<List<Vector3>>> followByCamera = [];
        int poseCount = 0;
        foreach (string jsonPath in Directory.EnumerateFiles(inputPath, "*.json"))
        {
            List<List<Vector3>> finalIndexListLeadAndFollow =
                JsonConvert.DeserializeObject<List<List<Vector3>>>(File.ReadAllText(jsonPath));

            if (jsonPath.Contains("lead"))
            {
                leadByCamera.Add(finalIndexListLeadAndFollow);
                if (finalIndexListLeadAndFollow[0].Count > poseCount)
                {
                    poseCount = finalIndexListLeadAndFollow[0].Count;
                }
            }
            else
            {
                followByCamera.Add(finalIndexListLeadAndFollow);
            }
        }

        int frameWithHighestConfidence = FrameWithHighestConfidence(leadByCamera, followByCamera);
        List<CameraSetup> cameras = [];
        int testingFrameNumber = 0;
        for (int i = 0; i < followByCamera.Count; i++)
        {
            List<Vector3> leadAtCamera = leadByCamera[i][frameWithHighestConfidence];
            List<Vector3> followAtCamera = followByCamera[i][frameWithHighestConfidence];

            // position cameras in a circle
            CameraSetup camera = new()
            {
                Size = imageSizes[i]
            };
            camera.AddPosesAndRecenterAndScaleToCamera(leadAtCamera, followAtCamera);

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

        float totalError = Calculate3DPosesAndTotalError(cameras, poseCount, testingFrameNumber);
        int iterationCount = 0;
        while (totalError > 10 && iterationCount < 1000000)
        {
            totalError = Iterate(
                cameras,
                poseCount,
                testingFrameNumber,
                totalError);
            iterationCount++;
        }

        Console.WriteLine(totalError);

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

    float Iterate(List<CameraSetup> cameras, int poseCount, int frameNumber, float lastIterationError)
    {
        float previousError = lastIterationError;
        const float errorMin = .02f;
        while (true)
        {
            float yawError = IterateYaw(cameras, poseCount, frameNumber, previousError, .05f);
            if (Math.Abs(yawError - previousError) < errorMin) break;
            previousError = yawError;
            Console.WriteLine("Yawing: " + previousError);
        }

        while (true)
        {
            float pitchError = IteratePitch(cameras, poseCount, frameNumber, previousError, .05f);
            if (Math.Abs(pitchError - previousError) < errorMin) break;
            previousError = pitchError;
            Console.WriteLine("Pitching: " + previousError);
        }

        while (true)
        {
            float focalError = IterateFocal(cameras, poseCount, frameNumber, previousError, .005f);
            if (Math.Abs(focalError - previousError) < errorMin * .1f) break;
            previousError = focalError;
            Console.WriteLine("Focusing: " + previousError);
        }

        while (true)
        {
            float rollError = IterateRoll(cameras, poseCount, frameNumber, previousError, .01f);
            if (Math.Abs(rollError - previousError) < errorMin) break;
            previousError = rollError;
            Console.WriteLine("Rolling: " + previousError);
        }

        while (true)
        {
            float xzPositionError =
                IterateXZPositionInCircleRelativeToOrigin(cameras, poseCount, frameNumber, previousError, .01f);
            if (Math.Abs(xzPositionError - previousError) < errorMin) break;
            previousError = xzPositionError;
            Console.WriteLine("Moving in XZ: " + previousError);
        }

        while (true)
        {
            float heightError = IterateHeightRelativeToOrigin(cameras, poseCount, frameNumber, previousError, .01f);
            if (Math.Abs(heightError - previousError) < errorMin) break;
            previousError = heightError;
            Console.WriteLine("Moving in Y: " + previousError);
        }

        while (true)
        {
            float radiusError =
                IterateRadiusWhilePreservingFocalAngle(cameras, poseCount, frameNumber, previousError, .01f);
            if (Math.Abs(radiusError - previousError) < errorMin) break;
            previousError = radiusError;
            Console.WriteLine("Moving in radius: " + previousError);
        }

        return previousError;
    }

    float IterateYaw(List<CameraSetup> cameras, int poseCount, int frameNumber, float lastError, float yawStepSize)
    {
        float thisError = lastError;
        for (int i = 0; i < cameras.Count; i++)
        {
            CameraSetup camera = cameras[i];

            // yaw camera left and right
            Quaternion originalRotation = camera.RotationsPerFrame[frameNumber];

            Quaternion leftYawRotation = camera.RotationsPerFrame[frameNumber] *
                                         Quaternion.CreateFromAxisAngle(camera.Up(frameNumber), yawStepSize);
            Quaternion rightYawRotation = camera.RotationsPerFrame[frameNumber] *
                                          Quaternion.CreateFromAxisAngle(camera.Up(frameNumber), -yawStepSize);

            camera.RotationsPerFrame[frameNumber] = leftYawRotation;
            float leftYawError = Calculate3DPosesAndTotalError(cameras, poseCount, frameNumber);

            camera.RotationsPerFrame[frameNumber] = rightYawRotation;
            float rightYawError = Calculate3DPosesAndTotalError(cameras, poseCount, frameNumber);

            if (leftYawError < rightYawError && leftYawError < thisError)
            {
                camera.RotationsPerFrame[frameNumber] = leftYawRotation;
                thisError = leftYawError;
            }
            else if (rightYawError < leftYawError && rightYawError < thisError)
            {
                camera.RotationsPerFrame[frameNumber] = rightYawRotation;
                thisError = rightYawError;
            }
            else
            {
                camera.RotationsPerFrame[frameNumber] = originalRotation;
            }
        }

        return thisError;
    }

    float IteratePitch(List<CameraSetup> cameras, int poseCount, int frameNumber, float lastError, float pitchStepSize)
    {
        float thisError = lastError;
        for (int i = 0; i < cameras.Count; i++)
        {
            CameraSetup camera = cameras[i];

            // pitch camera up and down
            Quaternion originalRotation = camera.RotationsPerFrame[frameNumber];

            Quaternion upPitchRotation = camera.RotationsPerFrame[frameNumber] *
                                         Quaternion.CreateFromAxisAngle(camera.Right(frameNumber), pitchStepSize);
            Quaternion downPitchRotation = camera.RotationsPerFrame[frameNumber] *
                                           Quaternion.CreateFromAxisAngle(camera.Right(frameNumber), -pitchStepSize);

            camera.RotationsPerFrame[frameNumber] = upPitchRotation;
            float upPitchError = Calculate3DPosesAndTotalError(cameras, poseCount, frameNumber);

            camera.RotationsPerFrame[frameNumber] = downPitchRotation;
            float downPitchError = Calculate3DPosesAndTotalError(cameras, poseCount, frameNumber);

            if (upPitchError < downPitchError && upPitchError < thisError)
            {
                camera.RotationsPerFrame[frameNumber] = upPitchRotation;
                thisError = upPitchError;
            }
            else if (downPitchError < upPitchError && downPitchError < thisError)
            {
                camera.RotationsPerFrame[frameNumber] = downPitchRotation;
                thisError = downPitchError;
            }
            else
            {
                camera.RotationsPerFrame[frameNumber] = originalRotation;
            }
        }

        return thisError;
    }

    float IterateFocal(List<CameraSetup> cameras, int poseCount, int frameNumber, float lastError, float focalStepSize)
    {
        float thisError = lastError;
        for (int i = 0; i < cameras.Count; i++)
        {
            CameraSetup camera = cameras[i];

            // adjust focal length
            float originalFocalLength = camera.FocalLength;

            camera.FocalLength = originalFocalLength + focalStepSize;
            float upFocalError = Calculate3DPosesAndTotalError(cameras, poseCount, frameNumber);

            camera.FocalLength = originalFocalLength - focalStepSize;
            float downFocalError = Calculate3DPosesAndTotalError(cameras, poseCount, frameNumber);

            if (upFocalError < downFocalError && upFocalError < thisError)
            {
                camera.FocalLength = originalFocalLength + focalStepSize;
                thisError = upFocalError;
            }
            else if (downFocalError < upFocalError && downFocalError < thisError)
            {
                camera.FocalLength = originalFocalLength - focalStepSize;
                thisError = downFocalError;
            }
            else
            {
                camera.FocalLength = originalFocalLength;
            }
        }

        return thisError;
    }

    float IterateRoll(List<CameraSetup> cameras, int poseCount, int frameNumber, float lastError, float rollStepSize)
    {
        float thisError = lastError;
        for (int i = 0; i < cameras.Count; i++)
        {
            CameraSetup camera = cameras[i];

            // roll camera left and right
            Quaternion originalRotation = camera.RotationsPerFrame[frameNumber];

            Quaternion leftRollRotation = camera.RotationsPerFrame[frameNumber] *
                                          Quaternion.CreateFromAxisAngle(camera.Forward(frameNumber), rollStepSize);
            Quaternion rightRollRotation = camera.RotationsPerFrame[frameNumber] *
                                           Quaternion.CreateFromAxisAngle(camera.Forward(frameNumber), -rollStepSize);

            camera.RotationsPerFrame[frameNumber] = leftRollRotation;
            float leftRollError = Calculate3DPosesAndTotalError(cameras, poseCount, frameNumber);

            camera.RotationsPerFrame[frameNumber] = rightRollRotation;
            float rightRollError = Calculate3DPosesAndTotalError(cameras, poseCount, frameNumber);

            if (leftRollError < rightRollError && leftRollError < thisError)
            {
                camera.RotationsPerFrame[frameNumber] = leftRollRotation;
                thisError = leftRollError;
            }
            else if (rightRollError < leftRollError && rightRollError < thisError)
            {
                camera.RotationsPerFrame[frameNumber] = rightRollRotation;
                thisError = rightRollError;
            }
            else
            {
                camera.RotationsPerFrame[frameNumber] = originalRotation;
            }
        }

        return thisError;
    }

    /// <summary>
    /// Rotate the cameras around the origin in a circle, in the clockwise direction, and counterclockwise direction.
    /// As the camera translates along the arc of the circle in the XZ dimension, it must also yaw to keep the target
    /// facing in front of it.
    /// </summary>
    float IterateXZPositionInCircleRelativeToOrigin(List<CameraSetup> cameras, int poseCount, int frameNumber,
        float lastError, float angleSize)
    {
        float thisError = lastError;
        for (int i = 0; i < cameras.Count; i++)
        {
            CameraSetup camera = cameras[i];
            Vector3 cameraOriginalPosition = camera.PositionsPerFrame[frameNumber];
            Quaternion cameraOriginalRotation = camera.RotationsPerFrame[frameNumber];

            // move camera in a circle around the origin
            float currentAngleRelativeToOrigin = MathF.Atan2(cameraOriginalPosition.Z, cameraOriginalPosition.X);
            float ccwAngle = currentAngleRelativeToOrigin + angleSize;
            Vector3 ccwPosition = new Vector3(
                MathF.Cos(ccwAngle) * MathF.Sqrt(cameraOriginalPosition.X * cameraOriginalPosition.X +
                                                 cameraOriginalPosition.Z * cameraOriginalPosition.Z),
                cameraOriginalPosition.Y,
                MathF.Sin(ccwAngle) * MathF.Sqrt(cameraOriginalPosition.X * cameraOriginalPosition.X +
                                                 cameraOriginalPosition.Z * cameraOriginalPosition.Z));
            camera.PositionsPerFrame[frameNumber] = ccwPosition;
            camera.RotationsPerFrame[frameNumber] = cameraOriginalRotation *
                                                    Quaternion.CreateFromAxisAngle(new Vector3(0, 1, 0),
                                                        -ccwAngle);
            float ccwError = Calculate3DPosesAndTotalError(cameras, poseCount, frameNumber);

            float cwAngle = currentAngleRelativeToOrigin - angleSize;
            Vector3 cwPosition = new Vector3(
                MathF.Cos(cwAngle) * MathF.Sqrt(cameraOriginalPosition.X * cameraOriginalPosition.X +
                                                cameraOriginalPosition.Z * cameraOriginalPosition.Z),
                cameraOriginalPosition.Y,
                MathF.Sin(cwAngle) * MathF.Sqrt(cameraOriginalPosition.X * cameraOriginalPosition.X +
                                                cameraOriginalPosition.Z * cameraOriginalPosition.Z));
            camera.PositionsPerFrame[frameNumber] = cwPosition;
            camera.RotationsPerFrame[frameNumber] = cameraOriginalRotation *
                                                    Quaternion.CreateFromAxisAngle(new Vector3(0, 1, 0),
                                                        cwAngle);
            float cwError = Calculate3DPosesAndTotalError(cameras, poseCount, frameNumber);

            if (ccwError < cwError && ccwError < thisError)
            {
                camera.PositionsPerFrame[frameNumber] = ccwPosition;
                camera.RotationsPerFrame[frameNumber] = cameraOriginalRotation *
                                                        Quaternion.CreateFromAxisAngle(new Vector3(0, 1, 0),
                                                            -ccwAngle);
                thisError = ccwError;
            }
            else if (cwError < ccwError && cwError < thisError)
            {
                camera.PositionsPerFrame[frameNumber] = cwPosition;
                camera.RotationsPerFrame[frameNumber] = cameraOriginalRotation *
                                                        Quaternion.CreateFromAxisAngle(new Vector3(0, 1, 0),
                                                            -cwAngle);
                thisError = cwError;
            }
            else
            {
                camera.PositionsPerFrame[frameNumber] = cameraOriginalPosition;
                camera.RotationsPerFrame[frameNumber] = cameraOriginalRotation;
            }
        }

        return thisError;
    }

    float IterateHeightRelativeToOrigin(List<CameraSetup> cameras, int poseCount, int frameNumber,
        float lastError, float angleSize)
    {
        float thisError = lastError;
        for (int i = 0; i < cameras.Count; i++)
        {
            CameraSetup camera = cameras[i];
            Vector3 cameraOriginalPosition = camera.PositionsPerFrame[frameNumber];
            Quaternion cameraOriginalRotation = camera.RotationsPerFrame[frameNumber];

            // move camera in an arc around the center of the circle
            float currentAngleRelativeToOrigin =
                MathF.Acos(Vector3.Dot(camera.Forward(frameNumber), cameraOriginalPosition));
            float angleRelativeToOrigin = currentAngleRelativeToOrigin + angleSize;
            Vector3 tallerPosition = cameraOriginalPosition with { Y = cameraOriginalPosition.Y + .1f };
            camera.PositionsPerFrame[frameNumber] = tallerPosition;
            camera.RotationsPerFrame[frameNumber] = cameraOriginalRotation *
                                                    Quaternion.CreateFromAxisAngle(new Vector3(1, 0, 0),
                                                        -angleRelativeToOrigin);
            float tallerError = Calculate3DPosesAndTotalError(cameras, poseCount, frameNumber);

            Vector3 shorterPosition = cameraOriginalPosition with { Y = cameraOriginalPosition.Y - .1f };
            camera.PositionsPerFrame[frameNumber] = shorterPosition;
            camera.RotationsPerFrame[frameNumber] = cameraOriginalRotation *
                                                    Quaternion.CreateFromAxisAngle(new Vector3(1, 0, 0),
                                                        angleRelativeToOrigin);
            float shorterError = Calculate3DPosesAndTotalError(cameras, poseCount, frameNumber);

            if (tallerError < shorterError && tallerError < thisError)
            {
                camera.PositionsPerFrame[frameNumber] = tallerPosition;
                camera.RotationsPerFrame[frameNumber] = cameraOriginalRotation *
                                                        Quaternion.CreateFromAxisAngle(new Vector3(1, 0, 0),
                                                            -angleRelativeToOrigin);
                thisError = tallerError;
            }
            else if (shorterError < tallerError && shorterError < thisError)
            {
                camera.PositionsPerFrame[frameNumber] = shorterPosition;
                camera.RotationsPerFrame[frameNumber] = cameraOriginalRotation *
                                                        Quaternion.CreateFromAxisAngle(new Vector3(1, 0, 0),
                                                            angleRelativeToOrigin);
                thisError = shorterError;
            }
            else
            {
                camera.PositionsPerFrame[frameNumber] = cameraOriginalPosition;
                camera.RotationsPerFrame[frameNumber] = cameraOriginalRotation;
            }
        }

        return thisError;
    }

    float IterateRadiusWhilePreservingFocalAngle(List<CameraSetup> cameras, int poseCount, int frameNumber,
        float lastError, float d)
    {
        float thisError = lastError;
        for (int i = 0; i < cameras.Count; i++)
        {
            CameraSetup camera = cameras[i];
            Vector3 cameraOriginalPosition = camera.PositionsPerFrame[frameNumber];
            float originalFocalLength = camera.FocalLength;

            // move camera toward the center, but simultaneously decrease the focal length to keep the focal angle the same
            float currentRadiusFromOrigin = cameraOriginalPosition.Length();
            float forwardRadius = currentRadiusFromOrigin - d;
            Vector3 forwardPosition = new Vector3(
                MathF.Cos(MathF.Atan2(cameraOriginalPosition.Z, cameraOriginalPosition.X)) * forwardRadius,
                cameraOriginalPosition.Y,
                MathF.Sin(MathF.Atan2(cameraOriginalPosition.Z, cameraOriginalPosition.X)) * forwardRadius);

            camera.FocalLength = originalFocalLength * cameraOriginalPosition.Length() / forwardPosition.Length();
            float forwardError = Calculate3DPosesAndTotalError(cameras, poseCount, frameNumber);

            float backwardRadius = currentRadiusFromOrigin + d;
            Vector3 backwardPosition = new Vector3(
                MathF.Cos(MathF.Atan2(cameraOriginalPosition.Z, cameraOriginalPosition.X)) * backwardRadius,
                cameraOriginalPosition.Y,
                MathF.Sin(MathF.Atan2(cameraOriginalPosition.Z, cameraOriginalPosition.X)) * backwardRadius);

            camera.FocalLength = originalFocalLength * cameraOriginalPosition.Length() / backwardPosition.Length();
            float backwardError = Calculate3DPosesAndTotalError(cameras, poseCount, frameNumber);

            if (forwardError < backwardError && forwardError < thisError)
            {
                camera.PositionsPerFrame[frameNumber] = forwardPosition;
                camera.FocalLength = originalFocalLength * cameraOriginalPosition.Length() / forwardPosition.Length();
                thisError = forwardError;
            }
            else if (backwardError < forwardError && backwardError < thisError)
            {
                camera.PositionsPerFrame[frameNumber] = backwardPosition;
                camera.FocalLength = originalFocalLength * cameraOriginalPosition.Length() / backwardPosition.Length();
                thisError = backwardError;
            }
            else
            {
                camera.PositionsPerFrame[frameNumber] = cameraOriginalPosition;
                camera.FocalLength = originalFocalLength;
            }
        }

        return thisError;
    }

    float Calculate3DPosesAndTotalError(List<CameraSetup> cameras, int poseCount, int frameNumber)
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
}