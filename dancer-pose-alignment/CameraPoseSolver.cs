﻿using System.ComponentModel;
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
                float error = cameraSetup.Error(merged3DPoseLeadPerFrame[0], merged3DPoseFollowPerFrame[0], 0);
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
            bool moved = highestErrorCam.TestSixTranslations(merged3DPoseLeadPerFrame[frameNumber],
                merged3DPoseFollowPerFrame[frameNumber], frameNumber);
            if (!moved)
            {
                break;
            }

            totalError = Calculate3DPosesAndTotalError();
            Console.WriteLine($"{iterationCount}:{totalError}");
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

    float Iterate(float lastIterationError)
    {
        float previousError = lastIterationError;
        const float errorMin = .02f;
        while (true)
        {
            float yawError = IterateYaw(.05f);
            if (Math.Abs(yawError - previousError) < errorMin) break;
            previousError = yawError;
            Console.WriteLine("Yawing: " + previousError);
        }

        while (true)
        {
            float pitchError = IteratePitch(.05f);
            if (Math.Abs(pitchError - previousError) < errorMin) break;
            previousError = pitchError;
            Console.WriteLine("Pitching: " + previousError);
        }

        while (true)
        {
            float focalError = IterateFocal(.005f);
            if (Math.Abs(focalError - previousError) < errorMin * .1f) break;
            previousError = focalError;
            Console.WriteLine("Focusing: " + previousError);
        }

        while (true)
        {
            float rollError = IterateRoll(.01f);
            if (Math.Abs(rollError - previousError) < errorMin) break;
            previousError = rollError;
            Console.WriteLine("Rolling: " + previousError);
        }

        while (true)
        {
            float xPositionError = IterateXPosition(.1f);
            if (Math.Abs(xPositionError - previousError) < errorMin) break;
            previousError = xPositionError;
            Console.WriteLine("Moving in X: " + previousError);
        }

        while (true)
        {
            float zPositionError = IterateZPosition(.1f);
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
            float heightError = IterateYPosition(.1f);
            if (Math.Abs(heightError - previousError) < errorMin) break;
            previousError = heightError;
            Console.WriteLine("Moving in Y: " + previousError);
        }

        return previousError;
    }

    float IterateYaw(float yawStepSize)
    {
        foreach (CameraSetup camera in cameras)
        {
            float currentError = Calculate3DPosesAndTotalError();

            // yaw camera left and right
            Quaternion originalRotation = camera.RotationsPerFrame[frameNumber];

            Quaternion leftYawRotation = originalRotation *
                                         Quaternion.CreateFromAxisAngle(Vector3.UnitY, yawStepSize);
            Quaternion rightYawRotation = originalRotation *
                                          Quaternion.CreateFromAxisAngle(Vector3.UnitY, -yawStepSize);

            camera.RotationsPerFrame[frameNumber] = leftYawRotation;
            float leftYawError = Calculate3DPosesAndTotalError();

            camera.RotationsPerFrame[frameNumber] = rightYawRotation;
            float rightYawError = Calculate3DPosesAndTotalError();

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

        return Calculate3DPosesAndTotalError();
    }

    float IteratePitch(float pitchStepSize)
    {
        foreach (CameraSetup camera in cameras)
        {
            float currentError = Calculate3DPosesAndTotalError();

            // pitch camera up and down
            Quaternion originalRotation = camera.RotationsPerFrame[frameNumber];
            Quaternion upPitchRotation = originalRotation *
                                         Quaternion.CreateFromAxisAngle(Vector3.UnitX, -pitchStepSize);

            // check if the angle between the up pitch forward vector and the ground forward vector is greater than 15 degrees
            Vector3 upPitchForward = Vector3.Normalize(Vector3.Transform(Vector3.UnitZ, upPitchRotation));

            float angleBetween = MathF.Acos(Vector3.Dot(upPitchForward, camera.Forward(frameNumber)));
            if (angleBetween > MathF.PI / 12)
            {
                Console.WriteLine("pitch angle too high" + angleBetween);
                camera.RotationsPerFrame[frameNumber] = originalRotation;
            }

            Quaternion downPitchRotation = originalRotation *
                                           Quaternion.CreateFromAxisAngle(Vector3.UnitZ, pitchStepSize);

            // check if the angle between the down pitch forward vector and the ground forward vector is greater than 15 degrees
            Vector3 downPitchForward = Vector3.Normalize(Vector3.Transform(Vector3.UnitZ, downPitchRotation));
            angleBetween = MathF.Acos(Vector3.Dot(downPitchForward, camera.Forward(frameNumber)));
            if (angleBetween > MathF.PI / 12)
            {
                Console.WriteLine("pitch angle too low" + angleBetween);
                camera.RotationsPerFrame[frameNumber] = originalRotation;
            }

            camera.RotationsPerFrame[frameNumber] = upPitchRotation;
            float upPitchError = Calculate3DPosesAndTotalError();
            if (AnyPointsHigherThan2pt5Meters() || LowestLeadAnkleIsMoreThan10CMAboveZero())
            {
                upPitchError = float.MaxValue;
            }

            camera.RotationsPerFrame[frameNumber] = downPitchRotation;
            float downPitchError = Calculate3DPosesAndTotalError();
            if (AnyPointsBelowGround())
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

        return Calculate3DPosesAndTotalError();
    }

    float IterateFocal(float focalStepSize)
    {
        foreach (CameraSetup camera in cameras)
        {
            float thisError = Calculate3DPosesAndTotalError();

            // adjust focal length
            float originalFocalLength = camera.FocalLength;

            camera.FocalLength = originalFocalLength + focalStepSize;
            if (camera.FocalLength > .6f)
            {
                Console.WriteLine($"focal length max: {cameras.IndexOf(camera)} + {camera.FocalLength}");
                camera.FocalLength = originalFocalLength;
            }

            float zoomInFocalError = Calculate3DPosesAndTotalError();

            camera.FocalLength = originalFocalLength - focalStepSize;
            if (camera.FocalLength < .001f)
            {
                Console.WriteLine($"focal length min: {cameras.IndexOf(camera)} + {camera.FocalLength}");
                camera.FocalLength = .001f;
            }

            float zoomOutFocalError = Calculate3DPosesAndTotalError();
            if (AnyPointsBelowGround())
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

        return Calculate3DPosesAndTotalError();
    }

    float IterateRoll(float rollStepSize)
    {
        foreach (CameraSetup camera in cameras)
        {
            float currentError = Calculate3DPosesAndTotalError();
            // roll camera left and right
            Quaternion originalRotation = camera.RotationsPerFrame[frameNumber];

            Quaternion leftRollRotation = originalRotation *
                                          Quaternion.CreateFromAxisAngle(Vector3.UnitZ, rollStepSize);

            // check if the up vector has rolled too far to the left relative to the ground up vector
            Vector3 leftRollUp = Vector3.Normalize(Vector3.Transform(Vector3.UnitY, leftRollRotation));

            float angleBetween = MathF.Acos(Vector3.Dot(leftRollUp, camera.Up(frameNumber)));
            if (angleBetween > MathF.PI / 12)
            {
                Console.WriteLine("roll angle too far to the left" + angleBetween);
                camera.RotationsPerFrame[frameNumber] = originalRotation;
            }

            Quaternion rightRollRotation = originalRotation *
                                           Quaternion.CreateFromAxisAngle(Vector3.UnitZ, -rollStepSize);

            // check if the up vector has rolled too far to the right relative to the ground up vector
            Vector3 rightRollUp = Vector3.Transform(Vector3.UnitY, rightRollRotation);
            angleBetween = MathF.Acos(Vector3.Dot(rightRollUp, camera.Up(frameNumber)));
            if (angleBetween > MathF.PI / 12)
            {
                Console.WriteLine("roll angle too far to the right" + angleBetween);
                camera.RotationsPerFrame[frameNumber] = originalRotation;
            }

            camera.RotationsPerFrame[frameNumber] = leftRollRotation;
            float leftRollError = Calculate3DPosesAndTotalError();

            camera.RotationsPerFrame[frameNumber] = rightRollRotation;
            float rightRollError = Calculate3DPosesAndTotalError();

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

        return Calculate3DPosesAndTotalError();
    }

    float IterateXPosition(float stepSize)
    {
        foreach (CameraSetup camera in cameras)
        {
            float currentError = Calculate3DPosesAndTotalError();
            Vector3 cameraOriginalPosition = camera.PositionsPerFrame[frameNumber];
            Quaternion cameraOriginalRotation = camera.RotationsPerFrame[frameNumber];
            float cameraOriginalFocalLength = camera.FocalLength;

            // move camera left
            Vector3 leftPosition = cameraOriginalPosition with { X = cameraOriginalPosition.X - stepSize };
            camera.PositionsPerFrame[frameNumber] = leftPosition;

            float translateLeftError = Calculate3DPosesAndTotalError();
            float lastLoopError = translateLeftError;
            float yawLeftCorrectionError = IterateYaw(.05f);
            while (yawLeftCorrectionError < lastLoopError && Math.Abs(yawLeftCorrectionError - lastLoopError) > .02f)
            {
                // recenter to make a fair comparison
                lastLoopError = yawLeftCorrectionError;
                yawLeftCorrectionError = IterateYaw(.05f);
            }

            lastLoopError = translateLeftError;
            float iterateLeftFocalError = IterateFocal(.005f);
            while (iterateLeftFocalError < lastLoopError && Math.Abs(iterateLeftFocalError - lastLoopError) > .02f)
            {
                // recenter to make a fair comparison
                lastLoopError = iterateLeftFocalError;
                iterateLeftFocalError = IterateFocal(.005f);
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

            float translateRightError = Calculate3DPosesAndTotalError();
            lastLoopError = translateRightError;
            float yawRightCorrectionError = IterateYaw(.05f);
            while (yawRightCorrectionError < lastLoopError && Math.Abs(yawRightCorrectionError - lastLoopError) > .02f)
            {
                // recenter to make a fair comparison
                lastLoopError = yawRightCorrectionError;
                yawRightCorrectionError = IterateYaw(.05f);
            }

            lastLoopError = translateRightError;
            float iterateRightFocalError = IterateFocal(.005f);
            while (iterateRightFocalError < lastLoopError && Math.Abs(iterateRightFocalError - lastLoopError) > .02f)
            {
                // recenter to make a fair comparison
                lastLoopError = iterateRightFocalError;
                iterateRightFocalError = IterateFocal(.005f);
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

        return Calculate3DPosesAndTotalError();
    }

    float IterateZPosition(float stepSize)
    {
        foreach (CameraSetup camera in cameras)
        {
            float currentError = Calculate3DPosesAndTotalError();
            Vector3 cameraOriginalPosition = camera.PositionsPerFrame[frameNumber];
            Quaternion cameraOriginalRotation = camera.RotationsPerFrame[frameNumber];
            float cameraOriginalFocalLength = camera.FocalLength;

            // move camera back
            Vector3 backPosition = cameraOriginalPosition with { Z = cameraOriginalPosition.Z - stepSize };
            camera.PositionsPerFrame[frameNumber] = backPosition;

            float translateBackError = Calculate3DPosesAndTotalError();
            float lastLoopError = translateBackError;
            float yawBackCorrectionError = IterateYaw(.05f);
            while (yawBackCorrectionError < lastLoopError && Math.Abs(yawBackCorrectionError - lastLoopError) > .02f)
            {
                // recenter to make a fair comparison
                lastLoopError = yawBackCorrectionError;
                yawBackCorrectionError = IterateYaw(.05f);
            }

            lastLoopError = translateBackError;
            float iterateBackFocalError = IterateFocal(.005f);
            while (iterateBackFocalError < lastLoopError && Math.Abs(iterateBackFocalError - lastLoopError) > .02f)
            {
                // recenter to make a fair comparison
                lastLoopError = iterateBackFocalError;
                iterateBackFocalError = IterateFocal(.005f);
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

            float translateForwardError = Calculate3DPosesAndTotalError();
            lastLoopError = translateForwardError;
            float yawForwardCorrectionError = IterateYaw(.05f);
            while (yawForwardCorrectionError < lastLoopError &&
                   Math.Abs(yawForwardCorrectionError - lastLoopError) > .02f)
            {
                // recenter to make a fair comparison
                lastLoopError = yawForwardCorrectionError;
                yawForwardCorrectionError = IterateYaw(.05f);
            }

            lastLoopError = translateForwardError;
            float iterateForwardFocalError = IterateFocal(.005f);
            while (iterateForwardFocalError < lastLoopError &&
                   Math.Abs(iterateForwardFocalError - lastLoopError) > .02f)
            {
                // recenter to make a fair comparison
                lastLoopError = iterateForwardFocalError;
                iterateForwardFocalError = IterateFocal(.005f);
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

        return Calculate3DPosesAndTotalError();
    }

    float IterateYPosition(float stepSize)
    {
        foreach (CameraSetup camera in cameras)
        {
            float currentError = Calculate3DPosesAndTotalError();
            Vector3 cameraOriginalPosition = camera.PositionsPerFrame[frameNumber];
            Quaternion cameraOriginalRotation = camera.RotationsPerFrame[frameNumber];

            Vector3 upPosition = cameraOriginalPosition;
            if (cameraOriginalPosition.Y + stepSize < 2.3f)
            {
                upPosition = cameraOriginalPosition with { Y = cameraOriginalPosition.Y + stepSize };
            }

            camera.PositionsPerFrame[frameNumber] = upPosition;

            // recenter pitch to make a fair comparison
            float moveUpError = Calculate3DPosesAndTotalError();
            float lastLoopError = moveUpError;
            float pitchUpCorrectionError = IteratePitch(.05f);
            while (pitchUpCorrectionError < lastLoopError && Math.Abs(pitchUpCorrectionError - lastLoopError) > .02f)
            {
                // recenter to make a fair comparison
                lastLoopError = pitchUpCorrectionError;
                pitchUpCorrectionError = IteratePitch(.05f);
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
            float pitchDownCorrectionError = IteratePitch(.05f);
            while (pitchDownCorrectionError < lastLoopError &&
                   Math.Abs(pitchDownCorrectionError - lastLoopError) > .02f)
            {
                // recenter to make a fair comparison
                lastLoopError = pitchDownCorrectionError;
                pitchDownCorrectionError = IteratePitch(.05f);
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

        return Calculate3DPosesAndTotalError();
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

    bool AnyPointsBelowGround() => merged3DPoseLeadPerFrame[frameNumber].Any(vec => vec.Y < 0) ||
                                   merged3DPoseFollowPerFrame[frameNumber].Any(vec => vec.Y < 0);

    bool LowestLeadAnkleIsMoreThan10CMAboveZero()
    {
        float lowestLeadAnkle = Math.Min(
            merged3DPoseLeadPerFrame[frameNumber][(int)Halpe.LAnkle].Y,
            merged3DPoseLeadPerFrame[frameNumber][(int)Halpe.RAnkle].Y);
        return lowestLeadAnkle > 0.1f;
    }

    bool AnyPointsHigherThan2pt5Meters()
    {
        return merged3DPoseLeadPerFrame[frameNumber].Any(vec => vec.Y > 2.5) ||
               merged3DPoseFollowPerFrame[frameNumber].Any(vec => vec.Y > 2.5);
    }

    public int GetFrameNumber() => frameNumber;
    public int GetTotalFrameCount() => totalFrameCount;
}