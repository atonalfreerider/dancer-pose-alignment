using System.Numerics;

namespace dancer_pose_alignment;

public class CameraSetup(
    string name,
    Vector2 size,
    int totalFrameCount,
    PoseType poseType,
    int startingFrame)
{
    // camera cylindrical position and intrinsic
    float radius = 3.5f;
    float height = 0;
    float alpha = 0;
    float focalLength = .05f;

    // cartesian position is constant (for now), rotation is tracked per frame
    public Vector3 Position => new(radius * MathF.Sin(alpha), height, radius * MathF.Cos(alpha));
    readonly Quaternion[] rotationsPerFrame = new Quaternion[totalFrameCount];

    // constants
    const float TorsoHeight = .4f; // used to determine height of camera from ground
    const float PixelToMeter = 0.000264583f;

    // unused, but collectively can be used to place other cameras at accurate radius
    public readonly List<Tuple<float, float>> CameraWall = [];

    Vector3 Forward(int frame) => Vector3.Transform(
        Vector3.UnitZ,
        rotationsPerFrame[frame]);

    Vector3 Up(int frame) => Vector3.Transform(
        Vector3.UnitY,
        rotationsPerFrame[frame]);

    Vector3 Right(int frame) => Vector3.Transform(
        Vector3.UnitX,
        rotationsPerFrame[frame]);

    // all poses in reference to image (x, -y, confidence) and camera center (x, y, confidence)
    readonly List<PoseBoundingBox>[] allPosesAndConfidencesPerFrame =
        new List<PoseBoundingBox>[totalFrameCount];

    readonly List<List<Vector3>>[] recenteredRescaledAllPosesPerFrame =
        new List<List<Vector3>>[totalFrameCount];
    
    Dictionary<int, Dictionary<int, List<Point>>> movedPoses = new();

    // indices referencing above pose lists
    readonly int[] leadIndicesPerFrame = new int[totalFrameCount];
    readonly int[] followIndicesPerFrame = new int[totalFrameCount];
    
    KalmanBoxTracker leadTracker;
    KalmanBoxTracker followTracker;

    // these poses are projected onto the image plane and used to calculate the 3D pose from ray projection
    readonly List<Vector3>[] leadProjectionsPerFrame = new List<Vector3>[totalFrameCount];
    readonly List<Vector3>[] followProjectionsPerFrame = new List<Vector3>[totalFrameCount];

    List<Vector3> affineTransforms; // all x,y motions and roll per frame, in pixels and radians

    /// <summary> 
    /// Called when poses are calculated for every frame 
    /// </summary> 
    public void SetAllPosesAtFrame(List<PoseBoundingBox> posesAtFrame, int frameNumber) 
    { 
        allPosesAndConfidencesPerFrame[frameNumber] = posesAtFrame; 
        recenteredRescaledAllPosesPerFrame[frameNumber] = posesAtFrame 
            .Select(poseBoundingBox => poseBoundingBox.Keypoints.Select(keypoint => 
                new Vector3( 
                    (keypoint.Point.X - size.X / 2) * PixelToMeter, 
                    -(keypoint.Point.Y - size.Y / 2) * PixelToMeter, // flip 
                    keypoint.Confidence)).ToList()).ToList(); // keep the confidence; 
    }

    public void SetAllAffine(List<Vector3> affine)
    {
        affineTransforms = affine;
    }

    public void FrameZeroLeadFollowFinderAndCamHeight(List<PoseBoundingBox> allPoses)
    {
        // find lead and follow
        int tallestIndex = -1;
        float tallestHeight = float.MinValue;
        int secondTallestIndex = -1;
        float secondTallestHeight = float.MinValue;
        int count = 0;
        foreach (PoseBoundingBox pose in allPoses)
        {
            float poseHeight = ExtremeHeight(pose);
            if (poseHeight > tallestHeight)
            {
                secondTallestHeight = tallestHeight;
                secondTallestIndex = tallestIndex;
                tallestHeight = poseHeight;
                tallestIndex = count;
            }
            else if (poseHeight > secondTallestHeight)
            {
                secondTallestHeight = poseHeight;
                secondTallestIndex = count;
            }

            count++;
        }

        leadIndicesPerFrame[0] = tallestIndex;
        followIndicesPerFrame[0] = secondTallestIndex;

        // set camera height based on what lead and background poses match levels, eg:
        // (1) if shoulders match shoulders, or I am a torso height above a squatter's shoulders, I am standing

        // (2) if lead hips match standing background person's hips, or squatting background person's shoulders,
        // my camera is squatting

        float leadRShoulderY =
            allPoses[tallestIndex].Keypoints[JointExtension.RShoulderIndex(poseType)].Point.Y;
        float leadRHipY = allPoses[tallestIndex].Keypoints[JointExtension.RHipIndex(poseType)].Point.Y;

        int standCount = 0;
        int sitCount = 0;
        count = 0;
        foreach (PoseBoundingBox pose in allPoses)
        {
            if (count == tallestIndex || count == secondTallestIndex)
            {
                continue;
            }

            bool backgroundFigureStanding = IsStanding(pose);
            float backgroundFigureShoulderY = pose.Keypoints[JointExtension.RShoulderIndex(poseType)].Point.Y;
            float backgroundFigureHipY = pose.Keypoints[JointExtension.RHipIndex(poseType)].Point.Y;

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

        height = standCount > sitCount ? 1.4f : .8f;
    }

    public void Match3DPoseToPoses(
        int frameNumber,
        IEnumerable<Vector3> lead3D,
        IEnumerable<Vector3> follow3D,
        int distanceLimit = 500)
    {
        if (allPosesAndConfidencesPerFrame[frameNumber] == null) return;

        // take the last 3d pose on this camera and match the profile to the closest pose here, within a threshold
        float lowestLeadError = float.MaxValue;
        int leadIndex = -1;

        float lowestFollowError = float.MaxValue;
        int followIndex = -1;

        float secondLowestFollowError = float.MaxValue;
        int secondFollowIndex = -1;

        int count = 0;
        foreach (PoseBoundingBox pose in allPosesAndConfidencesPerFrame[frameNumber])
        {
            float leadPoseError = PoseError(pose, lead3D, frameNumber);

            float followPoseError = PoseError(pose, follow3D, frameNumber);

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

        if (leadIndicesPerFrame[frameNumber] == -1 && lowestLeadError < distanceLimit)
        {
            leadIndicesPerFrame[frameNumber] = leadIndex;
        }

        if (followIndicesPerFrame[frameNumber] == -1 && lowestFollowError < distanceLimit)
        {
            followIndicesPerFrame[frameNumber] = followIndex;
        }
        
        leadTracker = new KalmanBoxTracker();
        leadTracker.Init(allPosesAndConfidencesPerFrame[frameNumber][leadIndex]);
        
        followTracker = new KalmanBoxTracker();
        followTracker.Init(allPosesAndConfidencesPerFrame[frameNumber][followIndex]);
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

        int count = 0;
        foreach (PoseBoundingBox pose in allPosesAndConfidencesPerFrame[frameNumber])
        {
            if (leadIndicesPerFrame[frameNumber] == count || followIndicesPerFrame[frameNumber] == count)
            {
                continue;
            }

            Vector2 poseRightAnkle = new(
                pose.Keypoints[JointExtension.RAnkleIndex(poseType)].Point.X,
                pose.Keypoints[JointExtension.RAnkleIndex(poseType)].Point.Y);

            Vector3? imgPtRayFloorIntersection = ImgPtRayFloorIntersection(poseRightAnkle);
            if (imgPtRayFloorIntersection == null)
            {
                continue;
            }

            float alpha = MathF.Atan2(imgPtRayFloorIntersection.Value.Z, imgPtRayFloorIntersection.Value.X) -
                          MathF.PI / 2; // unclear why off by 1/4 turn

            float poseTorsoHeightPixels = TorsoHeightPixels(pose);
            float poseTorsoHeight = poseTorsoHeightPixels * PixelToMeter;
            float poseAlphaFromGround = MathF.Atan2(poseTorsoHeight, focalLength);
            float poseRadius = Math.Abs(TorsoHeight / MathF.Tan(poseAlphaFromGround)) / 2; // arbitrary divide by 2

            CameraWall.Add(new Tuple<float, float>(alpha, poseRadius));
        }
    }

    #region USER MARKUP

    public Tuple<int, int> MarkDancer(Vector2 click, int frameNumber, string selectedButton)
    {
        (int closestIndex, int jointSelected) = GetClosestIndexAndJointSelected(click, frameNumber, []);

        switch (selectedButton)
        {
            case "Lead":
                leadIndicesPerFrame[frameNumber] = closestIndex; // select
                leadTracker = new KalmanBoxTracker();
                leadTracker.Init(allPosesAndConfidencesPerFrame[frameNumber][closestIndex]);
                break;
            case "Follow":
                followIndicesPerFrame[frameNumber] = closestIndex; // select
                followTracker = new KalmanBoxTracker();
                followTracker.Init(allPosesAndConfidencesPerFrame[frameNumber][closestIndex]);
                break;
            case "Move":
                // TODO create
                movedPoses[frameNumber][closestIndex][jointSelected] = new Point(
                    (int)click.X,
                    (int)click.Y); // move
                break;
        }

        return new Tuple<int, int>(closestIndex, jointSelected);
    }

    Tuple<int, int> GetClosestIndexAndJointSelected(Vector2 click, int frameNumber, ICollection<int> indicesToSkip)
    {
        int closestIndex = -1;
        int jointSelected = -1;
        float closestDistance = float.MaxValue;

        int count = 0;
        foreach (PoseBoundingBox pose in allPosesAndConfidencesPerFrame[frameNumber])
        {
            if (indicesToSkip.Contains(count))
            {
                continue;
            }

            int jointCount = 0;
            foreach (Keypoint joint in pose.Keypoints)
            {
                if (Vector2.Distance(click, new Vector2(joint.Point.X, joint.Point.Y)) < closestDistance)
                {
                    closestIndex = count;
                    jointSelected = jointCount;
                    closestDistance = Vector2.Distance(click, new Vector2(joint.Point.X, joint.Point.Y));
                }

                jointCount++;
            }

            count++;
        }

        return new Tuple<int, int>(closestIndex, jointSelected);
    }

    public void MoveKeypoint(Vector2 click, int frameNumber, Tuple<int, int> closestIndexAndJointSelected)
    {
        // TODO create
        movedPoses[frameNumber][closestIndexAndJointSelected.Item1][closestIndexAndJointSelected.Item2] = new Point(
            (int)click.X,
            (int)click.Y); // move
    }

    public void Unassign(int frameNumber)
    {
        leadIndicesPerFrame[frameNumber] = -1;
        followIndicesPerFrame[frameNumber] = -1;
    }

    #endregion

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
        Vector3 rescaledPt = new(
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
        Quaternion rotation = rotationsPerFrame[frame];
        for (int i = 0; i < adjustedKeypoints.Count; i++)
        {
            adjustedKeypoints[i] = Vector3.Transform(adjustedKeypoints[i] - Position, rotation) + Position;
        }

        // Translate keypoints to the camera's focal length 
        return adjustedKeypoints.Select(vec => vec + Forward(frame) * focalLength).ToList();
    }

    public Vector2 ReverseProjectPoint(Vector3 worldPoint, int frameNumber, bool overdraw = false)
    {
        Vector3 target = Vector3.Normalize(worldPoint - Position);
        Vector2 imagePlaneCoordinates = GetImagePlaneCoordinates(target, frameNumber);

        Vector2 offcenterAndRescaleAndFlip = new(
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
        float t = focalLength / Vector3.Dot(Forward(frameNumber), rayDirection);
        Vector3 intersectionPoint = t * rayDirection;

        // Calculate the coordinates relative to the image plane center
        Vector2 imagePlaneCoordinates = new(
            Vector3.Dot(intersectionPoint, Right(frameNumber)),
            Vector3.Dot(intersectionPoint, Up(frameNumber)));

        return imagePlaneCoordinates;
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
        Vector2 leadLeftAnkle = new(
            allPosesAndConfidencesPerFrame[0][leadIndicesPerFrame[0]].Keypoints[JointExtension.LAnkleIndex(poseType)].Point.X,
            allPosesAndConfidencesPerFrame[0][leadIndicesPerFrame[0]].Keypoints[JointExtension.LAnkleIndex(poseType)].Point.Y);

        Vector2 leadRightAnkle = new(
            allPosesAndConfidencesPerFrame[0][leadIndicesPerFrame[0]].Keypoints[JointExtension.RAnkleIndex(poseType)].Point.X,
            allPosesAndConfidencesPerFrame[0][leadIndicesPerFrame[0]].Keypoints[JointExtension.RAnkleIndex(poseType)].Point.Y);

        (bool isFacingLead, float leadPoseAnkleSlope) = FacingAndStanceSlope(leadRightAnkle, leadLeftAnkle);

        Vector3 stanceWidth = new(-.3f, 0f, 0f);

        // rotate camera in circle at 5m radius and 1.5m elevation pointed at origin until orientation and slope matches 
        float lowestAlpha = 0;
        float lowestDiff = float.MaxValue;
        for (float alpha = -MathF.PI; alpha < MathF.PI; alpha += .001f)
        {
            this.alpha = alpha;
            rotationsPerFrame[0] = Transform.LookAt(
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
        alpha = lowestAlpha;

        rotationsPerFrame[0] = Transform.LookAt(
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
        Vector2 leadRightAnkle = new(
            allPosesAndConfidencesPerFrame[0][leadIndicesPerFrame[0]].Keypoints[JointExtension.RAnkleIndex(poseType)].Point.X,
            allPosesAndConfidencesPerFrame[0][leadIndicesPerFrame[0]].Keypoints[JointExtension.RAnkleIndex(poseType)].Point.Y);

        const float hipHeight = .8f;
        float leadHipY = allPosesAndConfidencesPerFrame[0]
            [leadIndicesPerFrame[0]].Keypoints[JointExtension.RHipIndex(poseType)].Point.Y;

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
                focalLength += .001f;
                CenterRoll();
                CenterRightLeadAnkleOnOrigin(leadRightAnkle);
                CenterRoll();
                CenterRightLeadAnkleOnOrigin(leadRightAnkle);
            }
            else
            {
                focalLength -= .001f;
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
                rotationsPerFrame[0] *= Quaternion.CreateFromAxisAngle(Vector3.UnitY, .001f);
            }
            else
            {
                rotationsPerFrame[0] *= Quaternion.CreateFromAxisAngle(Vector3.UnitY, -.001f);
            }

            if (leadRightAnkle.Y > origin.Y)
            {
                // pitch up 
                rotationsPerFrame[0] *= Quaternion.CreateFromAxisAngle(Vector3.UnitX, -.001f);
            }
            else
            {
                // pitch down 
                rotationsPerFrame[0] *= Quaternion.CreateFromAxisAngle(Vector3.UnitX, .001f);
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
                rotationsPerFrame[0] *= Quaternion.CreateFromAxisAngle(Vector3.UnitZ, .001f);
            }
            else
            {
                // roll right 
                rotationsPerFrame[0] *= Quaternion.CreateFromAxisAngle(Vector3.UnitZ, -.001f);
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
        float alphaToSearch = alpha + MathF.PI;
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
            Console.WriteLine(name + " radius: " + radius + " -> " + newRadius);
            radius = newRadius;
        }
    }

    #endregion

    public void Update(int frameNumber)
    {
        List<PoseBoundingBox> detections = allPosesAndConfidencesPerFrame[frameNumber];
        PoseBoundingBox? leadTrackedPose = FindBestBoxFit(
            detections,
            leadTracker.Predict(),
            .3f);

        if (leadTrackedPose != null)
        {
            leadIndicesPerFrame[frameNumber] = detections.IndexOf(leadTrackedPose);
            leadTracker.Correct(leadTrackedPose);
        }
        
        PoseBoundingBox? followTrackedPose = FindBestBoxFit(
            detections,
            followTracker.Predict(),
            .3f);
        
        if(followTrackedPose != null)
        {
            followIndicesPerFrame[frameNumber] = detections.IndexOf(followTrackedPose);
            followTracker.Correct(followTrackedPose);
        }
    }
    
    /// <summary>
    /// Inverse over Union to find overlap between prediction and observation.
    /// </summary>
    static PoseBoundingBox? FindBestBoxFit(
        List<PoseBoundingBox> detections,
        Rectangle tracker,
        double iouThreshold)
    {
        PoseBoundingBox? highestIouDetection = null;
        double highestIou = iouThreshold;
        foreach (PoseBoundingBox detection in detections)
        {
            // find smallest intersection box
            double xx1 = Math.Max(detection.Bounds.Left, tracker.Left);
            double yy1 = Math.Max(detection.Bounds.Top, tracker.Top);
            double xx2 = Math.Min(detection.Bounds.Right, tracker.Right);
            double yy2 = Math.Min(detection.Bounds.Bottom, tracker.Bottom);
            double w = Math.Max(0.0, xx2 - xx1);
            double h = Math.Max(0.0, yy2 - yy1);
            double intersection = w * h;
            double detArea = detection.Bounds.Width * detection.Bounds.Height;
            double trkArea = tracker.Width * tracker.Height;
            double union = detArea + trkArea - intersection;
            double iou = intersection / union;
            if (iou > highestIou)
            {
                highestIou = iou;
                highestIouDetection = detection;
            }
        }

        return highestIouDetection;
    }

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
        Ray rayToJoint = new(
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
        // interpolate the frame
        int lastSampleFrame =
            (int)Math.Round(startingFrame + (frameNumber-1) * ((affineTransforms.Count - startingFrame) / (float)totalFrameCount));
        Vector3 lastAffine = affineTransforms[lastSampleFrame];
        int sampleFrame =
            (int)Math.Round(startingFrame + frameNumber * ((affineTransforms.Count - startingFrame) / (float)totalFrameCount));
        for (int i = lastSampleFrame + 1; i <= sampleFrame; i++)
        {
            lastAffine += affineTransforms[i];
        }
        
        float pitchAlpha = MathF.Atan2(lastAffine.Y, focalLength);
        float yawAlpha = MathF.Atan2(lastAffine.X, focalLength);

        rotationsPerFrame[frameNumber] = Quaternion.CreateFromAxisAngle(Vector3.UnitX, -pitchAlpha) *
                                         Quaternion.CreateFromAxisAngle(Vector3.UnitY, -yawAlpha) *
                                         rotationsPerFrame[frameNumber - 1];

        rotationsPerFrame[frameNumber] = rotationsPerFrame[frameNumber - 1];
    }

    float PoseError(PoseBoundingBox pose, IEnumerable<Vector3> pose3D, int frameNumber)
    {
        List<Vector2> reverseProjectedLead =
            pose3D.Select(vec => ReverseProjectPoint(vec, frameNumber, true)).ToList();

        return reverseProjectedLead.Select((target, i) => Vector2.Distance(
                target,
                new Vector2(pose.Keypoints[i].Point.X, pose.Keypoints[i].Point.Y)) * pose.Keypoints[i].Confidence)
            .Sum(); // multiply by confidence -> high confidence is high error
    }

    float TorsoHeightPixels(PoseBoundingBox pose)
    {
        float rHipY = pose.Keypoints[JointExtension.RHipIndex(poseType)].Point.Y;
        float rShoulderY = pose.Keypoints[JointExtension.RShoulderIndex(poseType)].Point.Y;

        return Math.Abs(rHipY - rShoulderY);
    }

    float ExtremeHeight(PoseBoundingBox pose)
    {
        float rAnkleY = pose.Keypoints[JointExtension.RAnkleIndex(poseType)].Point.Y;
        float lAnkleY = pose.Keypoints[JointExtension.LAnkleIndex(poseType)].Point.Y;

        float rShoulderY = pose.Keypoints[JointExtension.RShoulderIndex(poseType)].Point.Y;
        float lShoulderY = pose.Keypoints[JointExtension.LShoulderIndex(poseType)].Point.Y;

        return Math.Max(rAnkleY, lAnkleY) - Math.Min(rShoulderY, lShoulderY);
    }

    bool IsStanding(PoseBoundingBox pose)
    {
        float rAnkleY = pose.Keypoints[JointExtension.RAnkleIndex(poseType)].Point.Y;
        float rHipY = pose.Keypoints[JointExtension.RHipIndex(poseType)].Point.Y;
        float rShoulderY = pose.Keypoints[JointExtension.RShoulderIndex(poseType)].Point.Y;

        float torsoHeight = Math.Abs(rHipY - rShoulderY);
        float hipHeight = Math.Abs(rHipY - rAnkleY);

        float squatPrct = hipHeight / torsoHeight;

        return squatPrct > .8f;
    }

    Vector3? ImgPtRayFloorIntersection(Vector2 imgPt)
    {
        Vector3 projectedPoint = ProjectPoint(imgPt);
        Ray rayFromImgPoint = new(Position, Vector3.Normalize(projectedPoint - Position));

        return Transform.RayPlaneIntersection(new Plane(Vector3.UnitY, 0), rayFromImgPoint);
    }

    #endregion

    #region GETTERS

    public Tuple<int, int> GetLeadAndFollowIndexForFrame(int frameNumber)
    {
        return new Tuple<int, int>(leadIndicesPerFrame[frameNumber], followIndicesPerFrame[frameNumber]);
    }

    /// <summary>
    /// used for drawing
    /// </summary>
    public List<PoseBoundingBox> GetPosesPerDancerAtFrame(int frameNumber)
    {
        return allPosesAndConfidencesPerFrame[frameNumber];
    }

    #endregion
}