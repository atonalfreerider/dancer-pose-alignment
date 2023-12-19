using System.Numerics;

namespace dancer_pose_alignment;

public class CameraSetup(string name, Vector2 size, int totalFrameCount, PoseType poseType)
{
    public string Name = name;
    public float Radius = 3.5f;
    public float Height = 1.5f;
    public float Alpha = 0;
    public Vector3 Position => new Vector3(Radius * MathF.Sin(Alpha), Height, Radius * MathF.Cos(Alpha));
    
    public readonly Quaternion[] RotationsPerFrame = new Quaternion[totalFrameCount];
    public float FocalLength = .05f;

    const float PixelToMeter = 0.000264583f;

    public readonly Dictionary<string, List<Vector2>> ManualCameraPositionsByFrameByCamName = [];
    public readonly Dictionary<string, List<Tuple<int, int>>> ManualJointIndicesByFrameByCamName = [];

    public Vector3 Forward(int frame) => Vector3.Transform(
        Vector3.UnitZ,
        RotationsPerFrame[frame]);

    public Vector3 Up(int frame) => Vector3.Transform(
        Vector3.UnitY,
        RotationsPerFrame[frame]);

    public Vector3 Right(int frame) => Vector3.Transform(
        Vector3.UnitX,
        RotationsPerFrame[frame]);

    readonly List<Vector3>[] leadProjectionsPerFrame = new List<Vector3>[totalFrameCount];
    readonly List<Vector3>[] followProjectionsPerFrame = new List<Vector3>[totalFrameCount];

    readonly List<List<Vector3>>[] allPosesAndConfidencesPerFrame = new List<List<Vector3>>[totalFrameCount];
    readonly List<List<Vector3>>[] recenteredRescaledAllPosesPerFrame = new List<List<Vector3>>[totalFrameCount];

    readonly int[] leadIndicesPerFrame = new int[totalFrameCount];
    readonly int[] followIndicesPerFrame = new int[totalFrameCount];

    public List<Vector3> CurrentLead3DPose;
    public List<Vector3> CurrentFollow3DPose;
    Dictionary<string, Vector3> currentOtherCameraPositions = [];

    public void SetAllPosesAtFrame(List<List<Vector3>> allPoses, int frameNumber)
    {
        allPosesAndConfidencesPerFrame[frameNumber] = allPoses;
        recenteredRescaledAllPosesPerFrame[frameNumber] = allPoses.Select(pose => pose.Select(vec =>
                new Vector3(
                    (vec.X - size.X / 2) * PixelToMeter,
                    -(vec.Y - size.Y / 2) * PixelToMeter, // flip
                    vec.Z)) // keep the confidence
            .ToList()).ToList();

        if (frameNumber == 0)
        {
            int tallestIndex = -1;
            float tallestHeight = float.MinValue;
            int secondTallestIndex = -1;
            float secondTallestHeight = float.MinValue;
            foreach (List<Vector3> pose in allPoses)
            {
                float height = ExtremeHeight(pose);
                if (height > tallestHeight)
                {
                    secondTallestHeight = tallestHeight;
                    secondTallestIndex = tallestIndex;
                    tallestHeight = height;
                    tallestIndex = allPoses.IndexOf(pose);
                }
                else if (height > secondTallestHeight)
                {
                    secondTallestHeight = height;
                    secondTallestIndex = allPoses.IndexOf(pose);
                }
            }
            
            leadIndicesPerFrame[frameNumber] = tallestIndex;
            followIndicesPerFrame[frameNumber] = secondTallestIndex;
            
            Home();
        }
        else
        {
            // take the last 3d pose on this camera and match the profile to the closest pose here, within a threshold
            float lowestLeadError = float.MaxValue;
            int leadIndex = -1;

            float lowestFollowError = float.MaxValue;
            int followIndex = -1;

            int count = 0;
            foreach (List<Vector3> pose in allPoses)
            {
                float leadPoseError = PoseError(pose, true, frameNumber);

                float followPoseError = PoseError(pose, false, frameNumber);

                if (leadPoseError < lowestLeadError)
                {
                    leadIndex = count;
                    lowestLeadError = leadPoseError;
                }

                if (followPoseError < lowestFollowError)
                {
                    followIndex = count;
                    lowestFollowError = followPoseError;
                }

                count++;
            }

            if (leadIndex == followIndex)
            {
                followIndex = -1;
            }

            if (lowestLeadError < 1000)
            {
                leadIndicesPerFrame[frameNumber] = leadIndex;
            }

            if (lowestFollowError < 1000)
            {
                followIndicesPerFrame[frameNumber] = followIndex;
            }
        }
    }

    public Tuple<int, int> MarkDancer(Vector2 click, int frameNumber, string selectedButton)
    {
        int closestIndex = -1;
        int jointSelected = -1;
        float closestDistance = float.MaxValue;

        int counter = 0;
        foreach (List<Vector3> pose in allPosesAndConfidencesPerFrame[frameNumber])
        {
            foreach (Vector3 joint in pose)
            {
                if (Vector2.Distance(click, new Vector2(joint.X, joint.Y)) < closestDistance)
                {
                    closestIndex = counter;
                    jointSelected = pose.IndexOf(joint);
                    closestDistance = Vector2.Distance(click, new Vector2(joint.X, joint.Y));
                }
            }

            counter++;
        }

        switch (selectedButton)
        {
            case "Lead":
                leadIndicesPerFrame[frameNumber] = closestIndex; // select
                break;
            case "Follow":
                followIndicesPerFrame[frameNumber] = closestIndex; // select
                break;
            case "Move":
                allPosesAndConfidencesPerFrame[frameNumber][closestIndex][jointSelected] = new Vector3(
                    click.X,
                    click.Y,
                    allPosesAndConfidencesPerFrame[frameNumber][closestIndex][jointSelected].Z); // move
                break;
            default:
                // set camera position
                if (ManualCameraPositionsByFrameByCamName.TryGetValue(selectedButton, out List<Vector2> positionsByFrame))
                {
                    positionsByFrame[frameNumber] = click;
                }
                else
                {
                    ManualCameraPositionsByFrameByCamName[selectedButton] = new List<Vector2>(totalFrameCount);
                    for (int i = 0; i < totalFrameCount; i++)
                    {
                        ManualCameraPositionsByFrameByCamName[selectedButton].Add(Vector2.Zero);
                    }

                    ManualCameraPositionsByFrameByCamName[selectedButton][frameNumber] = click;
                }
                
                if(ManualJointIndicesByFrameByCamName.TryGetValue(selectedButton, out List<Tuple<int, int>> jointIndicesByFrame))
                {
                    jointIndicesByFrame[frameNumber] = new Tuple<int, int>(closestIndex, jointSelected);
                }
                else
                {
                    ManualJointIndicesByFrameByCamName[selectedButton] = new List<Tuple<int, int>>(totalFrameCount);
                    for (int i = 0; i < totalFrameCount; i++)
                    {
                        ManualJointIndicesByFrameByCamName[selectedButton].Add(new Tuple<int, int>(-1, -1));
                    }

                    ManualJointIndicesByFrameByCamName[selectedButton][frameNumber] = new Tuple<int, int>(closestIndex, jointSelected);
                }

                break;
                
        }

        return new Tuple<int, int>(closestIndex, jointSelected);
    }

    public void MoveKeypoint(Vector2 click, int frameNumber, Tuple<int, int> closestIndexAndJointSelected)
    {
        allPosesAndConfidencesPerFrame[frameNumber][closestIndexAndJointSelected.Item1][
            closestIndexAndJointSelected.Item2] = new Vector3(
            click.X,
            click.Y,
            1); // 100% confidence
    }

    public Tuple<int, int> LeadAndFollowIndexForFrame(int frameNumber)
    {
        return new Tuple<int, int>(leadIndicesPerFrame[frameNumber], followIndicesPerFrame[frameNumber]);
    }

    public List<List<Vector3>> PosesPerDancerAtFrame(int frameNumber)
    {
        return allPosesAndConfidencesPerFrame[frameNumber];
    }

    #region PROJECTION

    public void Project(int frameNumber)
    {
        // two things happening here: 
        // 1 the vector2 is transformed to a vector 3 where x and y on the image correspond to x and y on the 3D camera 
        // plane 
        // 2 the z confidence value is overwritten with 0, which is also the z value of the camera position 
        List<Vector3> flattenedLead = recenteredRescaledAllPosesPerFrame[frameNumber][leadIndicesPerFrame[frameNumber]]
            .Select(vec => vec with { Z = 0 }).ToList();
        List<Vector3> flattenedFollow =
            recenteredRescaledAllPosesPerFrame[frameNumber][followIndicesPerFrame[frameNumber]]
                .Select(vec => vec with { Z = 0 }).ToList();

        List<Vector3> leadProjectionsAtThisFrame = Adjusted(flattenedLead, frameNumber);
        leadProjectionsPerFrame[frameNumber] = leadProjectionsAtThisFrame;

        List<Vector3> followProjectionsAtThisFrame = Adjusted(flattenedFollow, frameNumber);
        followProjectionsPerFrame[frameNumber] = followProjectionsAtThisFrame;
    }

    Vector3 ProjectPoint(Vector2 imgPoint)
    {
        // rescale point
        Vector3 rescaledPt = new Vector3(
            (imgPoint.X - size.X / 2) * PixelToMeter,
            -(imgPoint.Y - size.Y / 2) * PixelToMeter, // flip
            0);
        
        List<Vector3> adjusted = Adjusted(new List<Vector3> { rescaledPt }, 0);
        return adjusted[0];
    }

    List<Vector3> Adjusted(IEnumerable<Vector3> keypoints, int frame)
    {
        // Translate keypoints to the camera center
        List<Vector3> adjustedKeypoints = keypoints.Select(vec => Position + vec).ToList();

        // Rotate keypoints around the camera center by the camera's rotation quaternion 
        Quaternion rotation = RotationsPerFrame[frame];
        for (int i = 0; i < adjustedKeypoints.Count; i++)
        {
            adjustedKeypoints[i] = Vector3.Transform(adjustedKeypoints[i] - Position, rotation) + Position;
        }

        // Translate keypoints to the camera's focal length 
        return adjustedKeypoints.Select(vec => vec + Forward(frame) * FocalLength).ToList();
    }

    public Vector2 ReverseProjectPoint(Vector3 worldPoint, int frameNumber, bool overdraw = false)
    {
        Vector3 target = TargetAtFrame(worldPoint);
        Vector2 imagePlaneCoordinates = GetImagePlaneCoordinates(target, frameNumber);

        Vector2 offcenterAndRescaleAndFlip = new Vector2(
            imagePlaneCoordinates.X / PixelToMeter + size.X / 2,
            -imagePlaneCoordinates.Y / PixelToMeter + size.Y / 2);

        if (!overdraw)
        {
            const float pad = 5;
            // don't render off screen
            if (offcenterAndRescaleAndFlip.Y < pad)
            {
                offcenterAndRescaleAndFlip.Y = pad;
            }

            if (offcenterAndRescaleAndFlip.Y > size.Y - pad)
            {
                offcenterAndRescaleAndFlip.Y = size.Y - pad;
            }

            if (offcenterAndRescaleAndFlip.X < pad)
            {
                offcenterAndRescaleAndFlip.X = pad;
            }

            if (offcenterAndRescaleAndFlip.X > size.X - pad)
            {
                offcenterAndRescaleAndFlip.X = size.X - pad;
            }
        }

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

    /// <summary>
    /// Provides a normalized vector from this camera to a target in 3D space at a given frame
    /// </summary>
    Vector3 TargetAtFrame(Vector3 vector3)
    {
        return Vector3.Normalize(vector3 - Position);
    }

    #endregion

    #region ITERATION

    /// <summary>
    /// Should only be called on frame 0
    /// </summary>
    public void Home()
    {
        if (leadIndicesPerFrame[0] == -1) return;
        
        // 1 - ORBIT 
        Vector2 leadLeftAnkle = new Vector2(
            allPosesAndConfidencesPerFrame[0][leadIndicesPerFrame[0]][JointExtension.LAnkleIndex(poseType)].X,
            allPosesAndConfidencesPerFrame[0][leadIndicesPerFrame[0]][JointExtension.LAnkleIndex(poseType)].Y);

        Vector2 leadRightAnkle = new Vector2(
            allPosesAndConfidencesPerFrame[0][leadIndicesPerFrame[0]][JointExtension.RAnkleIndex(poseType)].X,
            allPosesAndConfidencesPerFrame[0][leadIndicesPerFrame[0]][JointExtension.RAnkleIndex(poseType)].Y);

        (bool isFacingLead, float leadPoseAnkleSlope) = FacingAndStanceSlope(leadRightAnkle, leadLeftAnkle);
        
        Vector3 stanceWidth = new Vector3(-.3f, 0f, 0f);

        // rotate camera in circle at 5m radius and 1.5m elevation pointed at origin until orientation and slope matches 
        float lowestAlpha = 0;
        float lowestDiff = float.MaxValue;
        for (float alpha = 0; alpha < 2 * MathF.PI; alpha += .001f)
        {
            Alpha = alpha;
            RotationsPerFrame[0] = Transform.LookAt(
                Vector3.Zero,
                Quaternion.Identity,
                Position);

            Vector2 origin = ReverseProjectPoint(Vector3.Zero, 0, true);
            Vector2 leadStance = ReverseProjectPoint(stanceWidth, 0, true); // lead left ankle 
            
            (bool isFacingLeadInCamera, float slopeOfCamera) = FacingAndStanceSlope(origin, leadStance);

            if (isFacingLead == isFacingLeadInCamera)
            {
                float currentDiff = MathF.Abs(leadPoseAnkleSlope - slopeOfCamera);
                if(currentDiff< lowestDiff)
                {
                    lowestDiff = currentDiff;
                    lowestAlpha = alpha;
                }
            }
        }
        
        Alpha = lowestAlpha;

        RotationsPerFrame[0] = Transform.LookAt(
            Vector3.Zero,
            Quaternion.Identity,
            Position);

        CenterRoll();
        CenterRightLeadAnkleOnOrigin(leadRightAnkle);
        CenterRoll();
        CenterRightLeadAnkleOnOrigin(leadRightAnkle);

        HipLock();
    }

    static Tuple<bool, float> FacingAndStanceSlope(Vector2 leadRightAnkle, Vector2 leadLeftAnkle)
    {
        // calculate slope and orientation of lead ankle stance, so that iteration can match it 
        float leadPoseAnkleSlope = (leadLeftAnkle.Y - leadRightAnkle.Y) / (leadLeftAnkle.X - leadRightAnkle.X);
        bool isFacingLead = leadRightAnkle.X < leadLeftAnkle.X;
        if (!isFacingLead)
        {
            leadPoseAnkleSlope = (leadRightAnkle.Y - leadLeftAnkle.Y) / (leadRightAnkle.X - leadLeftAnkle.X);
        }

        return new Tuple<bool, float>(isFacingLead, leadPoseAnkleSlope);
    }

    public void HipLock()
    {
        Vector2 leadRightAnkle = new Vector2(
            allPosesAndConfidencesPerFrame[0][leadIndicesPerFrame[0]][JointExtension.RAnkleIndex(poseType)].X,
            allPosesAndConfidencesPerFrame[0][leadIndicesPerFrame[0]][JointExtension.RAnkleIndex(poseType)].Y);

        const float hipHeight = .8f;
        float leadHipY = allPosesAndConfidencesPerFrame[0]
            [leadIndicesPerFrame[0]][JointExtension.RHipIndex(poseType)].Y;

        CenterRoll();
        CenterRightLeadAnkleOnOrigin(leadRightAnkle);
        CenterRoll();
        CenterRightLeadAnkleOnOrigin(leadRightAnkle);

        float opticalHipHeight = ReverseProjectPoint(new Vector3(0, hipHeight, 0), 0, true).Y;

        int breaker = 0;
        while (Math.Abs(leadHipY - opticalHipHeight) > 1)
        {
            if (leadHipY < opticalHipHeight)
            {
                FocalLength += .001f;
                CenterRoll();
                CenterRightLeadAnkleOnOrigin(leadRightAnkle);
                CenterRoll();
                CenterRightLeadAnkleOnOrigin(leadRightAnkle);
            }
            else
            {
                FocalLength -= .001f;
                CenterRoll();
                CenterRightLeadAnkleOnOrigin(leadRightAnkle);
                CenterRoll();
                CenterRightLeadAnkleOnOrigin(leadRightAnkle);
            }

            opticalHipHeight = ReverseProjectPoint(new Vector3(0, hipHeight, 0), 0, true).Y;
            breaker++;
            if (breaker > 1000)
            {
                break;
            }
        }
    }

    void CenterRightLeadAnkleOnOrigin(Vector2 leadRightAnkle)
    {
        // yaw and pitch the camera until the origin is centered at the lead right ankle
        Vector2 origin = ReverseProjectPoint(Vector3.Zero, 0, true);

        int breaker = 0;
        while (Vector2.Distance(leadRightAnkle, origin) > 1)
        {
            if (leadRightAnkle.X < origin.X)
            {
                RotationsPerFrame[0] *= Quaternion.CreateFromAxisAngle(Vector3.UnitY, .001f);
            }
            else
            {
                RotationsPerFrame[0] *= Quaternion.CreateFromAxisAngle(Vector3.UnitY, -.001f);
            }

            if (leadRightAnkle.Y > origin.Y)
            {
                // pitch up 
                RotationsPerFrame[0] *= Quaternion.CreateFromAxisAngle(Vector3.UnitX, -.001f);
            }
            else
            {
                // pitch down 
                RotationsPerFrame[0] *= Quaternion.CreateFromAxisAngle(Vector3.UnitX, .001f);
            }

            origin = ReverseProjectPoint(Vector3.Zero, 0, true);
            breaker++;
            if (breaker > 1000)
            {
                break;
            }
        }
    }

    void CenterRoll()
    {
        Vector2 unitY = ReverseProjectPoint(Vector3.UnitY, 0, true);
        Vector2 origin = ReverseProjectPoint(Vector3.Zero, 0);

        int breaker = 0;
        while (Math.Abs(unitY.X - origin.X) > 1)
        {
            if (unitY.X < origin.X)
            {
                // roll left 
                RotationsPerFrame[0] *= Quaternion.CreateFromAxisAngle(Vector3.UnitZ, .001f);
            }
            else
            {
                // roll right 
                RotationsPerFrame[0] *= Quaternion.CreateFromAxisAngle(Vector3.UnitZ, -.001f);
            }

            origin = ReverseProjectPoint(Vector3.Zero, 0, true);
            unitY = ReverseProjectPoint(Vector3.UnitY, 0, true);

            breaker++;
            if (breaker > 1000)
            {
                break;
            }
        }
    }

    public Vector3? RayFloorIntersection(string otherCamName, float otherCamHeight)
    {
        Vector2 manual = ManualCameraPositionsByFrameByCamName[otherCamName][0];
        Vector3 projectedPoint = ProjectPoint(manual);
        Ray rayToManual = new Ray(Position, Vector3.Normalize(projectedPoint - Position));
        
        return Transform.RayXZPlaneIntersection(rayToManual, otherCamHeight);
    }
    
    public bool IterateOrientation(int frameNumber)
    {
        bool yawed = IterateYaw(0.01f, frameNumber);
        bool pitched = IteratePitch(0.01f, frameNumber);
        bool rolled = IterateRoll(0.01f, frameNumber);

        return yawed || pitched || rolled;
    }

    bool IterateYaw(float yawStepSize, int frameNumber)
    {
        float currentError = CurrentError(0);

        // yaw camera left and right
        Quaternion originalRotation = RotationsPerFrame[frameNumber];

        Quaternion leftYawRotation = originalRotation *
                                     Quaternion.CreateFromAxisAngle(Vector3.UnitY, yawStepSize);
        Quaternion rightYawRotation = originalRotation *
                                      Quaternion.CreateFromAxisAngle(Vector3.UnitY, -yawStepSize);

        RotationsPerFrame[frameNumber] = leftYawRotation;
        float leftYawError = CurrentError(0);

        RotationsPerFrame[frameNumber] = rightYawRotation;
        float rightYawError = CurrentError(0);

        if (leftYawError < rightYawError && leftYawError < currentError)
        {
            RotationsPerFrame[frameNumber] = leftYawRotation;
            return true;
        }

        if (rightYawError < leftYawError && rightYawError < currentError)
        {
            RotationsPerFrame[frameNumber] = rightYawRotation;
            return true;
        }

        // reset
        RotationsPerFrame[frameNumber] = originalRotation;
        return false;
    }

    bool IteratePitch(float pitchStepSize, int frameNumber)
    {
        float currentError = CurrentError(0);

        // pitch camera up and down
        Quaternion originalRotation = RotationsPerFrame[frameNumber];
        Quaternion upPitchRotation = originalRotation *
                                     Quaternion.CreateFromAxisAngle(Vector3.UnitX, -pitchStepSize);
        Quaternion downPitchRotation = originalRotation *
                                       Quaternion.CreateFromAxisAngle(Vector3.UnitZ, pitchStepSize);

        RotationsPerFrame[frameNumber] = upPitchRotation;
        float upPitchError = CurrentError(0);

        RotationsPerFrame[frameNumber] = downPitchRotation;
        float downPitchError = CurrentError(0);

        if (upPitchError < downPitchError && upPitchError < currentError)
        {
            RotationsPerFrame[frameNumber] = upPitchRotation;
            return true;
        }

        if (downPitchError < upPitchError && downPitchError < currentError)
        {
            RotationsPerFrame[frameNumber] = downPitchRotation;
            return true;
        }

        // reset
        RotationsPerFrame[frameNumber] = originalRotation;
        return false;
    }

    bool IterateRoll(float rollStepSize, int frameNumber)
    {
        float currentError = CurrentError(0);
        // roll camera left and right
        Quaternion originalRotation = RotationsPerFrame[frameNumber];
        Quaternion leftRollRotation = originalRotation *
                                      Quaternion.CreateFromAxisAngle(Vector3.UnitZ, rollStepSize);
        Quaternion rightRollRotation = originalRotation *
                                       Quaternion.CreateFromAxisAngle(Vector3.UnitZ, -rollStepSize);

        RotationsPerFrame[frameNumber] = leftRollRotation;
        float leftRollError = CurrentError(0);

        RotationsPerFrame[frameNumber] = rightRollRotation;
        float rightRollError = CurrentError(0);

        if (leftRollError < rightRollError && leftRollError < currentError)
        {
            RotationsPerFrame[frameNumber] = leftRollRotation;
            return true;
        }

        if (rightRollError < leftRollError && rightRollError < currentError)
        {
            RotationsPerFrame[frameNumber] = rightRollRotation;
            return true;
        }

        // reset
        RotationsPerFrame[frameNumber] = originalRotation;
        return false;
    }

    public float PoseError(IEnumerable<Vector3> pose3D, bool isLead, int frameNumber)
    {
        List<Vector3> comparePose = isLead
            ? recenteredRescaledAllPosesPerFrame[frameNumber][leadIndicesPerFrame[frameNumber]]
            : recenteredRescaledAllPosesPerFrame[frameNumber][followIndicesPerFrame[frameNumber]];

        return pose3D.Select(vec => ReverseProjectPoint(vec, frameNumber, true))
            .Select((target, i) => Vector2.Distance(
                target,
                new Vector2(comparePose[i].X, comparePose[i].Y)) * comparePose[i].Z).Sum();
    }

    public float CameraError(Dictionary<string, Vector3> otherCameras, int frameNumber)
    {
        currentOtherCameraPositions = otherCameras;
        float camError = 0;
        foreach ((string otherCamName, List<Vector2> manualPoints) in ManualCameraPositionsByFrameByCamName)
        {
            Vector2 camPoint = manualPoints[frameNumber];
            Vector3 otherCamPoint = otherCameras[otherCamName];
            Vector2 otherCamPoint2D = ReverseProjectPoint(otherCamPoint, frameNumber, true);
            camError += Vector2.Distance(camPoint, otherCamPoint2D);
        }

        return camError + StanceError() * 10;
    }

    float StanceError()
    {
        Vector2 leadLeftAnkle = new Vector2(
            allPosesAndConfidencesPerFrame[0][leadIndicesPerFrame[0]][JointExtension.LAnkleIndex(poseType)].X,
            allPosesAndConfidencesPerFrame[0][leadIndicesPerFrame[0]][JointExtension.LAnkleIndex(poseType)].Y);

        Vector2 leadRightAnkle = new Vector2(
            allPosesAndConfidencesPerFrame[0][leadIndicesPerFrame[0]][JointExtension.RAnkleIndex(poseType)].X,
            allPosesAndConfidencesPerFrame[0][leadIndicesPerFrame[0]][JointExtension.RAnkleIndex(poseType)].Y);

        (bool isFacingLead, float leadPoseAnkleSlope) = FacingAndStanceSlope(leadRightAnkle, leadLeftAnkle);
        
        Vector3 stanceWidth = new Vector3(-.3f, 0f, 0f);
        
        Vector2 origin = ReverseProjectPoint(Vector3.Zero, 0, true);
        Vector2 leadStance = ReverseProjectPoint(stanceWidth, 0, true); // lead left ankle 
            
        (bool isFacingLeadInCamera, float slopeOfCamera) = FacingAndStanceSlope(origin, leadStance);

        if (isFacingLead == isFacingLeadInCamera)
        {
            return MathF.Abs(leadPoseAnkleSlope - slopeOfCamera);
        }

        return float.MaxValue;
    }

    #endregion

    #region REFERENCE

    public bool HasPoseAtFrame(int frameNumber, bool isLead)
    {
        return isLead
            ? recenteredRescaledAllPosesPerFrame[frameNumber][leadIndicesPerFrame[frameNumber]].Count > 0
            : recenteredRescaledAllPosesPerFrame[frameNumber][followIndicesPerFrame[frameNumber]].Count > 0;
    }

    public Ray PoseRay(int frameNumber, int jointNumber, bool isLead)
    {
        Ray rayToJoint = new Ray(
            Position,
            Vector3.Normalize(isLead
                ? leadProjectionsPerFrame[frameNumber][jointNumber] - Position
                : followProjectionsPerFrame[frameNumber][jointNumber] - Position));
        return rayToJoint;
    }

    public float JointConfidence(int frameNumber, int jointNumber, bool isLead)
    {
        return isLead
            ? recenteredRescaledAllPosesPerFrame[frameNumber][leadIndicesPerFrame[frameNumber]][jointNumber].Z
            : recenteredRescaledAllPosesPerFrame[frameNumber][followIndicesPerFrame[frameNumber]][jointNumber].Z;
    }

    public void CopyRotationToNextFrame(int frameNumber)
    {
        RotationsPerFrame[frameNumber] = RotationsPerFrame[frameNumber - 1];
    }

    float CurrentError(int frameNumber)
    {
        return PoseError(allPosesAndConfidencesPerFrame[frameNumber][leadIndicesPerFrame[frameNumber]], true,
                   frameNumber) +
               PoseError(allPosesAndConfidencesPerFrame[frameNumber][followIndicesPerFrame[frameNumber]], true,
                   frameNumber) +
               CameraError();
    }

    float PoseError(IReadOnlyList<Vector3> pose, bool isLead, int frameNumber)
    {
        if (isLead)
        {
            List<Vector2> reverseProjectedLead =
                CurrentLead3DPose.Select(vec => ReverseProjectPoint(vec, frameNumber, true)).ToList();

            return reverseProjectedLead.Select((target, i) => Vector2.Distance(
                target,
                new Vector2(pose[i].X, pose[i].Y)) * pose[i].Z).Sum();
        }

        List<Vector2> reverseProjectedFollow =
            CurrentFollow3DPose.Select(vec => ReverseProjectPoint(vec, frameNumber, true)).ToList();

        return reverseProjectedFollow.Select((target, i) => Vector2.Distance(
            target,
            new Vector2(pose[i].X, pose[i].Y)) * pose[i].Z).Sum();
    }

    float CameraError()
    {
        return CameraError(currentOtherCameraPositions, 0);
    }
    
    public float PoseHeight(List<Vector3> pose)
    {
        float h = 0;
        Vector3 rAnkle = allPosesAndConfidencesPerFrame[0][leadIndicesPerFrame[0]][JointExtension.RAnkleIndex(poseType)];
        Vector3 rKnee = allPosesAndConfidencesPerFrame[0][leadIndicesPerFrame[0]][JointExtension.RKneeIndex(poseType)];
        Vector3 rHip = allPosesAndConfidencesPerFrame[0][leadIndicesPerFrame[0]][JointExtension.RHipIndex(poseType)];
        Vector3 rShoulder = allPosesAndConfidencesPerFrame[0][leadIndicesPerFrame[0]][JointExtension.RShoulderIndex(poseType)];
        
        Vector2 rAnkle2D = new Vector2(rAnkle.X, rAnkle.Y);
        Vector2 rKnee2D = new Vector2(rKnee.X, rKnee.Y);
        Vector2 rHip2D = new Vector2(rHip.X, rHip.Y);
        Vector2 rShoulder2D = new Vector2(rShoulder.X, rShoulder.Y);
        
        h += Vector2.Distance(rAnkle2D, rKnee2D);
        h += Vector2.Distance(rKnee2D, rHip2D);
        h += Vector2.Distance(rHip2D, rShoulder2D);
        return h;
    }
    
    float ExtremeHeight(IReadOnlyList<Vector3> pose)
    {
        Vector3 rAnkle =pose[JointExtension.RAnkleIndex(poseType)];
        Vector3 lAnkle = pose[JointExtension.LAnkleIndex(poseType)];
        
        Vector3 rShoulder = pose[JointExtension.RShoulderIndex(poseType)];
        Vector3 lShoulder = pose[JointExtension.LShoulderIndex(poseType)];
        
        return Math.Max(rAnkle.Y, lAnkle.Y) - Math.Min(rShoulder.Y, lShoulder.Y);
    }

    #endregion
}