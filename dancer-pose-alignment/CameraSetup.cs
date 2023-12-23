using System.Numerics;

namespace dancer_pose_alignment;

public class CameraSetup(
    string name,
    Vector2 size,
    int totalFrameCount,
    PoseType poseType,
    int startingFrame)
{
    public string Name = name;
    public float Radius = 3.5f;
    float Height = 0;
    public List<float> HeightsSetByOtherCameras = [];
    public float Alpha = 0;
    const float TorsoHeight = .4f;
    public Vector3 Position => new(Radius * MathF.Sin(Alpha), Height, Radius * MathF.Cos(Alpha));

    public readonly Quaternion[] RotationsPerFrame = new Quaternion[totalFrameCount];
    public float FocalLength = .05f;

    const float PixelToMeter = 0.000264583f;

    public readonly Dictionary<string, List<CameraHandAnchor>> ManualCameraPositionsByFrameByCamName = [];

    public readonly List<Tuple<float, float>> CameraWall = [];

    Vector3 Forward(int frame) => Vector3.Transform(
        Vector3.UnitZ,
        RotationsPerFrame[frame]);

    Vector3 Up(int frame) => Vector3.Transform(
        Vector3.UnitY,
        RotationsPerFrame[frame]);

    Vector3 Right(int frame) => Vector3.Transform(
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

    /// <summary>
    /// Called when poses are being calculated every frame
    /// </summary>
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
            FrameZeroLeadFollowFinderAndCamHeight(allPoses);
        }
        else
        {
            Match3DPoseToPoses(frameNumber);
        }
    }

    public void SetAllPosesForEveryFrame(List<List<List<Vector3>>> posesByFrame)
    {
        for (int i = 0; i < totalFrameCount; i++)
        {
            int sampleFrame = (int)Math.Round(startingFrame + i * ((posesByFrame.Count - startingFrame) / (float)totalFrameCount));
            allPosesAndConfidencesPerFrame[i] = posesByFrame[sampleFrame];
            recenteredRescaledAllPosesPerFrame[i] = posesByFrame[sampleFrame].Select(pose => pose.Select(vec =>
                    new Vector3(
                        (vec.X - size.X / 2) * PixelToMeter,
                        -(vec.Y - size.Y / 2) * PixelToMeter, // flip
                        vec.Z)) // keep the confidence
                .ToList()).ToList();

            if (i == 0)
            {
                FrameZeroLeadFollowFinderAndCamHeight(posesByFrame[sampleFrame]);
            }
        }
    }

    void FrameZeroLeadFollowFinderAndCamHeight(List<List<Vector3>> allPoses)
    {
        // find lead and follow
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

        leadIndicesPerFrame[0] = tallestIndex;
        followIndicesPerFrame[0] = secondTallestIndex;

        // set camera height based on what lead and background poses match levels, eg:
        // (1) if shoulders match shoulders, or I am a torso height above a squatter's shoulders, I am standing

        // (2) if lead hips match standing background person's hips, or squatting background person's shoulders,
        // my camera is squatting

        float leadRShoulderY =
            allPoses[tallestIndex][JointExtension.RShoulderIndex(poseType)].Y;
        float leadRHipY = allPoses[tallestIndex][JointExtension.RHipIndex(poseType)].Y;

        int count = 0;
        int standCount = 0;
        int sitCount = 0;
        foreach (List<Vector3> pose in allPoses)
        {
            if (count == tallestIndex || count == secondTallestIndex)
            {
                count++;
                continue;
            }

            bool backgroundFigureStanding = IsStanding(pose);
            float backgroundFigureShoulderY = pose[JointExtension.RShoulderIndex(poseType)].Y;
            float backgroundFigureHipY = pose[JointExtension.RHipIndex(poseType)].Y;

            if (backgroundFigureStanding)
            {
                if (backgroundFigureShoulderY < leadRShoulderY || // standing shoulder is above lead shoulder
                    Math.Abs(leadRShoulderY - backgroundFigureShoulderY) <
                    Math.Abs(leadRHipY - backgroundFigureHipY))
                {
                    // standing shoulder and lead shoulder are square
                    standCount++;
                }
                else
                {
                    // standing shoulder is closer to lead hip
                    sitCount++;
                }
            }
            else
            {
                if (backgroundFigureShoulderY < leadRHipY)
                {
                    // sitting shoulder is above lead hip
                    standCount++;
                }
                else
                {
                    // sitting shoulder is below lead hip
                    sitCount++;
                }
            }

            count++;
        }

        Height = standCount > sitCount ? 1.4f : .8f;
    }

    public void Unassign(int frameNumber)
    {
        leadIndicesPerFrame[frameNumber] = -1;
        followIndicesPerFrame[frameNumber] = -1;
    }

    public void Match3DPoseToPoses(int frameNumber, int distanceLimit = 500)
    {
        // take the last 3d pose on this camera and match the profile to the closest pose here, within a threshold
        float lowestLeadError = float.MaxValue;
        int leadIndex = -1;

        float lowestFollowError = float.MaxValue;
        int followIndex = -1;

        float secondLowestFollowError = float.MaxValue;
        int secondFollowIndex = -1;

        int count = 0;
        foreach (List<Vector3> pose in allPosesAndConfidencesPerFrame[frameNumber])
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
                secondFollowIndex = followIndex;
                secondLowestFollowError = lowestFollowError;
                followIndex = count;
                lowestFollowError = followPoseError;
            }
            else if (followPoseError < secondLowestFollowError)
            {
                secondFollowIndex = count;
                secondLowestFollowError = followPoseError;
            }

            count++;
        }

        if (leadIndex == followIndex)
        {
            followIndex = secondFollowIndex;
        }

        if (lowestLeadError < distanceLimit)
        {
            leadIndicesPerFrame[frameNumber] = leadIndex;
        }

        if (lowestFollowError < distanceLimit)
        {
            followIndicesPerFrame[frameNumber] = followIndex;
        }
    }

    /// <summary>
    /// This technique is a middle ground. Knowing that the initial camera radii are set to 3.5m, having the height
    /// calculated, and the alpha calculated, then the zoom, it is possible to see the other side of the circle and
    /// infer the radius of each pose, and also the alpha of each pose based on the img pt projection. Once this wall
    /// is established, by reassigning the radii, an even more accurate wall can be established, and so on.
    /// </summary>
    public void CalculateCameraWall(int frameNumber)
    {
        if (leadIndicesPerFrame[frameNumber] == -1) return;

        // take each right ankle, find the alpha angle from the 3D lead forward, and calculate the radius based on the
        // height of the pose
        CameraWall.Clear();

        int counter = 0;
        foreach (List<Vector3> pose in allPosesAndConfidencesPerFrame[frameNumber])
        {
            if (leadIndicesPerFrame[frameNumber] == counter || followIndicesPerFrame[frameNumber] == counter)
            {
                counter++;
                continue;
            }

            Vector2 poseRightAnkle = new Vector2(
                pose[JointExtension.RAnkleIndex(poseType)].X,
                pose[JointExtension.RAnkleIndex(poseType)].Y);

            Vector3? imgPtRayFloorIntersection = ImgPtRayFloorIntersection(poseRightAnkle);
            if (imgPtRayFloorIntersection == null)
            {
                counter++;
                continue;
            }

            float alpha = MathF.Atan2(imgPtRayFloorIntersection.Value.Z, imgPtRayFloorIntersection.Value.X) -
                          MathF.PI / 2; // unclear why off by 1/4 turn

            float poseTorsoHeightPixels = TorsoHeightPixels(pose);
            float poseTorsoHeight = poseTorsoHeightPixels * PixelToMeter;
            float poseAlphaFromGround = MathF.Atan2(poseTorsoHeight, FocalLength);
            float poseRadius = Math.Abs(TorsoHeight / MathF.Tan(poseAlphaFromGround)) / 2; // arbitrary divide by 2

            CameraWall.Add(new Tuple<float, float>(alpha, poseRadius));
            counter++;
        }
    }

    public Tuple<int, int> MarkDancer(Vector2 click, int frameNumber, string selectedButton)
    {
        (int closestIndex, int jointSelected) = GetClosestIndexAndJointSelected(click, frameNumber, []);

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
                if (leadIndicesPerFrame[frameNumber] == closestIndex ||
                    followIndicesPerFrame[frameNumber] == closestIndex)
                {
                    (closestIndex, jointSelected) = GetClosestIndexAndJointSelected(click, frameNumber,
                        [leadIndicesPerFrame[frameNumber], followIndicesPerFrame[frameNumber]]);
                }

                if (ManualCameraPositionsByFrameByCamName.TryGetValue(selectedButton,
                        out List<CameraHandAnchor> positionsByFrame))
                {
                    positionsByFrame[frameNumber] = new CameraHandAnchor(click, closestIndex, jointSelected);
                }
                else
                {
                    ManualCameraPositionsByFrameByCamName[selectedButton] = new List<CameraHandAnchor>(totalFrameCount);
                    for (int i = 0; i < totalFrameCount; i++)
                    {
                        ManualCameraPositionsByFrameByCamName[selectedButton]
                            .Add(new CameraHandAnchor(Vector2.Zero, -1, -1));
                    }

                    ManualCameraPositionsByFrameByCamName[selectedButton][frameNumber] =
                        new CameraHandAnchor(click, closestIndex, jointSelected);
                    ;
                }

                break;
        }

        return new Tuple<int, int>(closestIndex, jointSelected);
    }

    Tuple<int, int> GetClosestIndexAndJointSelected(Vector2 click, int frameNumber, ICollection<int> indicesToSkip)
    {
        int closestIndex = -1;
        int jointSelected = -1;
        float closestDistance = float.MaxValue;

        int counter = 0;
        foreach (List<Vector3> pose in allPosesAndConfidencesPerFrame[frameNumber])
        {
            if (indicesToSkip.Contains(counter))
            {
                counter++;
                continue;
            }

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
        if (leadIndicesPerFrame[frameNumber] != -1)
        {
            List<Vector3> flattenedLead =
                recenteredRescaledAllPosesPerFrame[frameNumber][leadIndicesPerFrame[frameNumber]]
                    .Select(vec => vec with { Z = 0 }).ToList();

            List<Vector3> leadProjectionsAtThisFrame = Adjusted(flattenedLead, frameNumber);
            leadProjectionsPerFrame[frameNumber] = leadProjectionsAtThisFrame;
        }

        if (followIndicesPerFrame[frameNumber] != -1)
        {
            List<Vector3> flattenedFollow =
                recenteredRescaledAllPosesPerFrame[frameNumber][followIndicesPerFrame[frameNumber]]
                    .Select(vec => vec with { Z = 0 }).ToList();

            List<Vector3> followProjectionsAtThisFrame = Adjusted(flattenedFollow, frameNumber);
            followProjectionsPerFrame[frameNumber] = followProjectionsAtThisFrame;
        }
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
        for (float alpha = -MathF.PI; alpha < MathF.PI; alpha += .001f)
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
                if (currentDiff < lowestDiff)
                {
                    lowestDiff = currentDiff;
                    lowestAlpha = alpha;
                }
            }
        }

        // convert back to alpha using atan2
        Alpha = lowestAlpha;

        RotationsPerFrame[0] = Transform.LookAt(
            Vector3.Zero,
            Quaternion.Identity,
            Position);

        HipLock();

        CalculateCameraWall(0);
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

    void HipLock()
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

    public void SetRadiusFromCameraWall(List<Tuple<float, float>> allCameraWalls)
    {
        // set the radius from the camera wall
        if (allCameraWalls.Count == 0) return;

        int closestAlphaIndex = -1;
        float closestAlphaDistance = float.MaxValue;
        int count = 0;
        float alphaToSearch = Alpha + MathF.PI;
        if (alphaToSearch > MathF.PI)
        {
            alphaToSearch -= 2 * MathF.PI;
        }

        foreach (Tuple<float, float> alphaAndRadius in allCameraWalls)
        {
            float alpha = alphaAndRadius.Item1;
            float alphaDistance = Math.Abs(alpha - alphaToSearch);
            if (alphaDistance < closestAlphaDistance)
            {
                closestAlphaDistance = alphaDistance;
                closestAlphaIndex = count;
            }

            count++;
        }

        float newRadius = allCameraWalls[closestAlphaIndex].Item2;

        if (newRadius is > 1 and < 10)
        {
            Console.WriteLine(Name + " radius: " + Radius + " -> " + newRadius);
            Radius = newRadius;
        }
    }

    Vector3? ImgPtRayFloorIntersection(Vector2 imgPt)
    {
        Vector3 projectedPoint = ProjectPoint(imgPt);
        Ray rayFromImgPoint = new Ray(Position, Vector3.Normalize(projectedPoint - Position));

        return Transform.RayPlaneIntersection(new Plane(Vector3.UnitY, 0), rayFromImgPoint);
    }
 
    #endregion

    #region REFERENCE

    public bool HasPoseAtFrame(int frameNumber, bool isLead)
    {
        if (isLead && leadIndicesPerFrame[frameNumber] == -1)
        {
            return false;
        }

        if (!isLead && followIndicesPerFrame[frameNumber] == -1)
        {
            return false;
        }

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
    
    public void TrackRotationFromLastFrame(int frameNumber)
    {
        List<List<Vector3>> backgroundRecenteredFromLast = [];
        int count = 0;
        foreach (List<Vector3> vector3s in recenteredRescaledAllPosesPerFrame[frameNumber -1])
        {
            if (count == leadIndicesPerFrame[frameNumber - 1] || count == followIndicesPerFrame[frameNumber - 1])
            {
                count++;
                continue;
            }
            
            backgroundRecenteredFromLast.Add(vector3s);
            count++;
        }
        
        List<List<Vector3>> backgroundRecenteredFromThis = [];
        count = 0;
        foreach (List<Vector3> vector3s in recenteredRescaledAllPosesPerFrame[frameNumber])
        {
            if (count == leadIndicesPerFrame[frameNumber] || count == followIndicesPerFrame[frameNumber])
            {
                count++;
                continue;
            }
            
            backgroundRecenteredFromThis.Add(vector3s);
            count++;
        }
        
        Vector2 meanMotionVector = Vector2.Zero;
        List<Vector3> allMotionVectors = [];
        foreach (List<Vector3> lastPose in backgroundRecenteredFromLast)
        {
            for (int i = 0; i < JointExtension.PoseCount(PoseType.Coco); i++)
            {
                int closestJointIndex = -1;
                float closestJointDistance = float.MaxValue;
                Vector3 lastJoint = lastPose[i];
                foreach (List<Vector3> currentPose in backgroundRecenteredFromThis)
                {
                    Vector3 thisJoint = currentPose[i];
                    float distance = Vector2.Distance(
                        new Vector2(thisJoint.X, thisJoint.Y),
                        new Vector2(lastJoint.X, lastJoint.Y)); // lower is better
                    
                    float confidencePenalty = (1 - thisJoint.Z) * (1 - lastJoint.Z); // lower is better
                    if (distance * confidencePenalty < closestJointDistance)
                    {
                        closestJointDistance = distance * confidencePenalty;
                        closestJointIndex = backgroundRecenteredFromThis.IndexOf(currentPose);
                    }
                }

                Vector3 closestJoint = backgroundRecenteredFromThis[closestJointIndex][i];
                Vector3 motionVector = new Vector3(
                    closestJoint.X - lastJoint.X,
                    closestJoint.Y - lastJoint.Y,
                    closestJointDistance);
                allMotionVectors.Add(motionVector);
            }
        }

        allMotionVectors = allMotionVectors.OrderByDescending(vec => vec.Z).Reverse().ToList();
        // remove second half of list
        allMotionVectors = allMotionVectors.Take(allMotionVectors.Count / 2).ToList();
        meanMotionVector = new Vector2(
            allMotionVectors.Select(vec => vec.X).Sum() / allMotionVectors.Count,
            allMotionVectors.Select(vec => vec.Y).Sum() / allMotionVectors.Count);
        
        float pitchAlpha = MathF.Atan2(meanMotionVector.Y, FocalLength);
        float yawAlpha = MathF.Atan2(meanMotionVector.X, FocalLength);
        
        RotationsPerFrame[frameNumber] = Quaternion.CreateFromAxisAngle(Vector3.UnitX, -pitchAlpha) *
                                         Quaternion.CreateFromAxisAngle(Vector3.UnitY, -yawAlpha) *
                                         RotationsPerFrame[frameNumber - 1];
    }
    
    float PoseError(IReadOnlyList<Vector3> pose, bool isLead, int frameNumber)
    {
        if (isLead)
        {
            List<Vector2> reverseProjectedLead =
                CurrentLead3DPose.Select(vec => ReverseProjectPoint(vec, frameNumber, true)).ToList();

            return reverseProjectedLead.Select((target, i) => Vector2.Distance(
                target,
                new Vector2(pose[i].X, pose[i].Y)) * pose[i].Z).Sum(); // multiply by confidence -> high confidence is high error
        }

        List<Vector2> reverseProjectedFollow =
            CurrentFollow3DPose.Select(vec => ReverseProjectPoint(vec, frameNumber, true)).ToList();

        return reverseProjectedFollow.Select((target, i) => Vector2.Distance(
            target,
            new Vector2(pose[i].X, pose[i].Y)) * pose[i].Z).Sum();
    }

    public float PoseHeight(int index, float camYImgComponent)
    {
        List<Vector3> pose = allPosesAndConfidencesPerFrame[0][index];

        Vector3 rAnkle = pose[JointExtension.RAnkleIndex(poseType)];
        Vector3 rHip = pose[JointExtension.RHipIndex(poseType)];
        Vector3 rShoulder = pose[JointExtension.RShoulderIndex(poseType)];

        float torsoHeight = Math.Abs(rHip.Y - rShoulder.Y);
        float hipHeight = Math.Abs(rHip.Y - rAnkle.Y);

        float squatPrct = hipHeight / torsoHeight;

        float totalHeight = TorsoHeight + .9f * squatPrct;

        float camHeight = Math.Abs(rAnkle.Y - camYImgComponent);
        float ankleToShoulder = Math.Abs(rAnkle.Y - rShoulder.Y);

        return totalHeight * camHeight / ankleToShoulder;
    }

    float TorsoHeightPixels(IReadOnlyList<Vector3> pose)
    {
        Vector3 rHip = pose[JointExtension.RHipIndex(poseType)];
        Vector3 rShoulder = pose[JointExtension.RShoulderIndex(poseType)];

        return Math.Abs(rHip.Y - rShoulder.Y);
    }

    float ExtremeHeight(IReadOnlyList<Vector3> pose)
    {
        Vector3 rAnkle = pose[JointExtension.RAnkleIndex(poseType)];
        Vector3 lAnkle = pose[JointExtension.LAnkleIndex(poseType)];

        Vector3 rShoulder = pose[JointExtension.RShoulderIndex(poseType)];
        Vector3 lShoulder = pose[JointExtension.LShoulderIndex(poseType)];

        return Math.Max(rAnkle.Y, lAnkle.Y) - Math.Min(rShoulder.Y, lShoulder.Y);
    }

    bool IsStanding(IReadOnlyList<Vector3> pose)
    {
        Vector3 rAnkle = pose[JointExtension.RAnkleIndex(poseType)];
        Vector3 rHip = pose[JointExtension.RHipIndex(poseType)];
        Vector3 rShoulder = pose[JointExtension.RShoulderIndex(poseType)];

        float torsoHeight = Math.Abs(rHip.Y - rShoulder.Y);
        float hipHeight = Math.Abs(rHip.Y - rAnkle.Y);

        float squatPrct = hipHeight / torsoHeight;

        return squatPrct > .8f;
    }

    #endregion

    public class CameraHandAnchor(Vector2 imgPosition, int poseIndex, int jointIndex)
    {
        public Vector2 ImgPosition = imgPosition;
        public int PoseIndex = poseIndex;
        public int JointIndex = jointIndex;
    }
}