using System.Data.SQLite;
using System.Numerics;
using Newtonsoft.Json;

namespace dancer_pose_alignment;

public class CameraSetup(
    string name,
    Vector2 size,
    int totalFrameCountAt30Fps,
    PoseType poseType,
    int startingFrame,
    int maxFrame,
    string dbPath)
{
    // camera cylindrical position and intrinsic
    float radius = 3.5f;
    float height = 0;
    float alpha = 0;
    float focalLength = .05f;

    // cartesian position is constant (for now), rotation is tracked per frame
    public Vector3 Position => new(radius * MathF.Sin(alpha), height, radius * MathF.Cos(alpha));
    readonly Quaternion[] rotationsPerFrame = new Quaternion[totalFrameCountAt30Fps];

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
        new List<PoseBoundingBox>[totalFrameCountAt30Fps];

    Dictionary<int, Dictionary<int, List<Point>>> movedPoses = new();

    KalmanBoxTracker? leadTracker;
    KalmanBoxTracker? followTracker;

    // these poses are projected onto the image plane and used to calculate the 3D pose from ray projection
    readonly List<Vector3>[] leadProjectionsPerFrame = new List<Vector3>[totalFrameCountAt30Fps];
    readonly List<Vector3>[] followProjectionsPerFrame = new List<Vector3>[totalFrameCountAt30Fps];

    List<Vector3> affineTransforms; // all x,y motions and roll per frame, in pixels and radians

    /// <summary> 
    /// Called when poses are calculated for every frame 
    /// </summary> 
    public void SetAllPosesAtFrame(int frameNumber)
    {
        int sampleFrame = SampleFrame(frameNumber);
        List<PoseBoundingBox> posesAtFrame = PosesAtFrameFromDb(dbPath, FilePrefix(), sampleFrame);

        foreach (PoseBoundingBox poseBoundingBox in posesAtFrame)
        {
            poseBoundingBox.RecenterdKeypoints =  poseBoundingBox.Keypoints.Select(keypoint =>new Vector3(
                (keypoint.Point.X - size.X / 2) * PixelToMeter,
                -(keypoint.Point.Y - size.Y / 2) * PixelToMeter, // flip 
                keypoint.Confidence)).ToList(); // keep the confidence; 
        }
        
        allPosesAndConfidencesPerFrame[frameNumber] = posesAtFrame;
        
        if (frameNumber == 0)
        {
            FrameZeroLeadFollowFinderAndCamHeight(posesAtFrame);
        }
        
        PoseBoundingBox? leadPose = LeadPose(frameNumber);
        PoseBoundingBox? followPose = FollowPose(frameNumber);
        if (leadTracker == null && leadPose != null)
        {
            leadTracker = new KalmanBoxTracker();
            leadTracker.Init(leadPose);
        }
        
        if (followTracker == null && followPose != null)
        {
            followTracker = new KalmanBoxTracker();
            followTracker.Init(followPose);
        }
    }

    static List<PoseBoundingBox> PosesAtFrameFromDb(string dbPath, string filePrefix, int sampleFrame)
    {
        List<PoseBoundingBox> posesAtFrame = [];
        using SQLiteConnection conn = new($"URI=file:{dbPath}");
        conn.Open();

        string query = $"""
                        SELECT
                             id,
                             keypoints,
                             bounds,
                             track_id
                        FROM table_{filePrefix}
                        WHERE frame = @frameNumber
                        """;

        using SQLiteCommand cmd = new(query, conn);
        cmd.Parameters.AddWithValue("@frameNumber", sampleFrame);

        using SQLiteDataReader reader = cmd.ExecuteReader();
        while (reader.Read())
        {
            int dbId = reader.GetInt32(0);
            string keypoints = reader.GetString(1);
            string bounds = reader.GetString(2);
            int trackId = reader.GetInt32(3);

            // Assuming PoseBoundingBox has a constructor that takes these values
            PoseBoundingBox pose = new()
            {
                DbId = dbId,
                Keypoints = JsonConvert.DeserializeObject<List<Keypoint>>(keypoints),
                Bounds = JsonConvert.DeserializeObject<Rectangle>(bounds),
                Class = new Class(trackId, "person")
            };
            posesAtFrame.Add(pose);
        }

        return posesAtFrame;
    }

    public void SetAllAffine(List<Vector3> affine)
    {
        affineTransforms = affine;
    }

    void FrameZeroLeadFollowFinderAndCamHeight(List<PoseBoundingBox> allPoses)
    {
        // find lead and follow
        PoseBoundingBox tallestIndex = allPoses.First();
        float tallestHeight = float.MinValue;
        PoseBoundingBox secondTallestIndex = allPoses.Last();
        float secondTallestHeight = float.MinValue;
        foreach (PoseBoundingBox pose in allPoses)
        {
            float poseHeight = ExtremeHeight(pose);
            if (poseHeight > tallestHeight)
            {
                secondTallestHeight = tallestHeight;
                secondTallestIndex = tallestIndex;
                tallestHeight = poseHeight;
                tallestIndex = pose;
            }
            else if (poseHeight > secondTallestHeight)
            {
                secondTallestHeight = poseHeight;
                secondTallestIndex = pose;
            }
        }

        PoseBoundingBox? leadPose = LeadPose(0);
        if (leadPose == null)
        {
            tallestIndex.Class.Id = 0;
        }

        PoseBoundingBox? followPose = FollowPose(0);
        if (followPose == null)
        {
            secondTallestIndex.Class.Id = 1;
        }

        UpdateTrackIds(0);

        // set camera height based on what lead and background poses match levels, eg:
        // (1) if shoulders match shoulders, or I am a torso height above a squatter's shoulders, I am standing

        // (2) if lead hips match standing background person's hips, or squatting background person's shoulders,
        // my camera is squatting

        float leadRShoulderY =
            tallestIndex.Keypoints[JointExtension.RShoulderIndex(poseType)].Point.Y;
        float leadRHipY = tallestIndex.Keypoints[JointExtension.RHipIndex(poseType)].Point.Y;

        int standCount = 0;
        int sitCount = 0;
        foreach (PoseBoundingBox pose in allPoses)
        {
            if (pose.Class.Id is 0 or 1)
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
        }

        height = standCount > sitCount ? 1.4f : .8f;
    }

    public void Match3DPoseToPoses(
        int frameNumber,
        List<Vector3> lead3D,
        List<Vector3> follow3D,
        int distanceLimit = 500)
    {
        // take the last 3d pose on this camera and match the profile to the closest pose here, within a threshold
        float lowestLeadError = float.MaxValue;
        PoseBoundingBox matchLeadPose = allPosesAndConfidencesPerFrame[frameNumber].First();

        float lowestFollowError = float.MaxValue;
        PoseBoundingBox matchFollowPose = allPosesAndConfidencesPerFrame[frameNumber].Last();

        float secondLowestFollowError = float.MaxValue;
        PoseBoundingBox? matchSecondFollowPose = null;
        
        foreach (PoseBoundingBox pose in allPosesAndConfidencesPerFrame[frameNumber])
        {
            float leadPoseError = PoseError(pose, lead3D, frameNumber);

            float followPoseError = PoseError(pose, follow3D, frameNumber);

            if (leadPoseError < lowestLeadError)
            {
                matchLeadPose = pose;
                lowestLeadError = leadPoseError;
            }

            if (followPoseError < lowestFollowError)
            {
                matchSecondFollowPose = matchFollowPose;
                secondLowestFollowError = lowestFollowError;
                matchFollowPose = pose;
                lowestFollowError = followPoseError;
            }
            else if (followPoseError < secondLowestFollowError)
            {
                matchSecondFollowPose = pose;
                secondLowestFollowError = followPoseError;
            }
        }

        if (matchLeadPose.DbId == matchFollowPose.DbId && matchSecondFollowPose != null)
        {
            matchFollowPose = matchSecondFollowPose;
        }

        PoseBoundingBox? leadPose = LeadPose(frameNumber);
        if (leadPose == null && lowestLeadError < distanceLimit)
        {
            matchLeadPose.Class.Id = 0;
        }

        PoseBoundingBox? followPose = FollowPose(frameNumber);
        if (followPose == null && lowestFollowError < distanceLimit)
        {
            matchFollowPose.Class.Id = 1;
        }
        
        UpdateTrackIds(frameNumber);

        leadTracker = new KalmanBoxTracker();
        leadTracker.Init(LeadPose(frameNumber)!);

        followTracker = new KalmanBoxTracker();
        followTracker.Init(FollowPose(frameNumber)!);
    }

    /// <summary>
    /// This technique is a middle ground. Knowing that the initial camera radii are set to 3.5m, having the height
    /// calculated, and the alpha calculated, then the zoom, it is possible to see the other side of the circle and
    /// infer the radius of each pose, and also the alpha of each pose based on the img pt projection. Once this wall
    /// is established, by reassigning the radii, an even more accurate wall can be established, and so on.
    /// </summary>
    public void CalculateCameraWall(int frameNumber)
    {
        PoseBoundingBox? leadPose = LeadPose(frameNumber);
        if (leadPose == null) return;

        // take each right ankle, find the alpha angle from the 3D lead forward, and calculate the radius based on the
        // height of the pose
        CameraWall.Clear();

        int count = 0;
        foreach (PoseBoundingBox pose in allPosesAndConfidencesPerFrame[frameNumber])
        {
            if (pose.Class.Id == count)
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

            count++;
        }
    }

    #region USER MARKUP

    public Tuple<PoseBoundingBox, int> MarkDancer(Vector2 click, int frameNumber, string selectedButton)
    {
        (PoseBoundingBox closestPose, int jointSelected) = GetClosestIndexAndJointSelected(click, frameNumber);

        switch (selectedButton)
        {
            case "Lead":
                foreach (PoseBoundingBox poseBoundingBox in allPosesAndConfidencesPerFrame[frameNumber])
                {
                    if (poseBoundingBox.Class.Id == 0)
                    {
                        poseBoundingBox.Class.Id = -1;
                    }
                }
                closestPose.Class.Id = 0;
                UpdateTrackIds(frameNumber);
                leadTracker = new KalmanBoxTracker();
                leadTracker.Init(closestPose);
                break;
            case "Follow":
                foreach (PoseBoundingBox poseBoundingBox in allPosesAndConfidencesPerFrame[frameNumber])
                {
                    if (poseBoundingBox.Class.Id == 1)
                    {
                        poseBoundingBox.Class.Id = -1;
                    }
                }
                closestPose.Class.Id = 1;
                UpdateTrackIds(frameNumber);
                followTracker = new KalmanBoxTracker();
                followTracker.Init(closestPose);
                break;
            case "Move":
                // TODO create

                break;
        }

        return new Tuple<PoseBoundingBox, int>(closestPose, jointSelected);
    }

    Tuple<PoseBoundingBox, int> GetClosestIndexAndJointSelected(Vector2 click, int frameNumber)
    {
        PoseBoundingBox closestPose = allPosesAndConfidencesPerFrame[frameNumber].First();
        int jointSelected = -1;
        float closestDistance = float.MaxValue;
        
        foreach (PoseBoundingBox pose in allPosesAndConfidencesPerFrame[frameNumber])
        {
            int jointCount = 0;
            foreach (Keypoint joint in pose.Keypoints)
            {
                if (Vector2.Distance(click, new Vector2(joint.Point.X, joint.Point.Y)) < closestDistance)
                {
                    closestPose = pose;
                    jointSelected = jointCount;
                    closestDistance = Vector2.Distance(click, new Vector2(joint.Point.X, joint.Point.Y));
                }

                jointCount++;
            }
        }

        return new Tuple<PoseBoundingBox, int>(closestPose, jointSelected);
    }

    public void MoveKeypoint(Vector2 click, int frameNumber, Tuple<PoseBoundingBox?, int> closestIndexAndJointSelected)
    {
        // TODO create
    }

    public void Unassign(int frameNumber)
    {
        foreach (PoseBoundingBox poseBoundingBox in allPosesAndConfidencesPerFrame[frameNumber])
        {
            poseBoundingBox.Class.Id = -1;
        }
        UpdateTrackIds(frameNumber);
    }

    #endregion

    #region PROJECTION

    public void Project(int frameNumber)
    {
        // two things happening here: 
        // 1 the vector2 is transformed to a vector 3 where x and y on the image correspond to x and y on the 3D camera 
        // plane 
        // 2 the z confidence value is overwritten with 0, which is also the z value of the camera position 
        PoseBoundingBox? leadPose = LeadPose(frameNumber);
        if (leadPose != null)
        {
            List<Vector3> flattenedLead = leadPose.RecenterdKeypoints
                .Select(vec => vec with { Z = 0 }).ToList();

            List<Vector3> leadProjectionsAtThisFrame = Adjusted(flattenedLead, frameNumber);
            leadProjectionsPerFrame[frameNumber] = leadProjectionsAtThisFrame;
        }

        PoseBoundingBox? followPose = FollowPose(frameNumber);
        if (followPose != null)
        {
            List<Vector3> flattenedFollow = followPose.RecenterdKeypoints
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
        PoseBoundingBox? leadPose = LeadPose(0);
        if (leadPose == null) return;

        // 1 - ORBIT 
        
        Vector2 leadLeftAnkle = new(
            leadPose.Keypoints[JointExtension.LAnkleIndex(poseType)].Point.X,
            leadPose.Keypoints[JointExtension.LAnkleIndex(poseType)].Point.Y);

        Vector2 leadRightAnkle = new(
            leadPose.Keypoints[JointExtension.RAnkleIndex(poseType)].Point.X,
            leadPose.Keypoints[JointExtension.RAnkleIndex(poseType)].Point.Y);

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
        PoseBoundingBox? leadPose = LeadPose(0);
        if (leadPose == null) return;
        
        Vector2 leadRightAnkle = new(
            leadPose.Keypoints[JointExtension.RAnkleIndex(poseType)].Point.X,
            leadPose.Keypoints[JointExtension.RAnkleIndex(poseType)].Point.Y);

        const float hipHeight = .8f;
        float leadHipY = leadPose.Keypoints[JointExtension.RHipIndex(poseType)].Point.Y;

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

    #region KALMAN
    
    public void Update(int frameNumber)
    {
        if (leadTracker == null || followTracker == null) return;
        
        const float IouThreshold = .6f;
        List<PoseBoundingBox> detections = allPosesAndConfidencesPerFrame[frameNumber];
        PoseBoundingBox? leadDetection = FindBestBoxFit(
            detections,
            leadTracker.Predict(),
            IouThreshold);
        
        if (leadDetection != null)
        {
            foreach (PoseBoundingBox poseBoundingBox in detections)
            {
                if (poseBoundingBox.Class.Id == 0)
                {
                    poseBoundingBox.Class.Id = -1;
                }
            }
            leadDetection.Class.Id = 0;
            leadTracker.Correct(leadDetection);
        }
        else
        {
            leadTracker = null;
        }

        PoseBoundingBox? followDetection = FindBestBoxFit(
            detections,
            followTracker.Predict(),
            IouThreshold);

        if (leadDetection != null && followDetection != null && followDetection.DbId == leadDetection.DbId)
        {
            followDetection = null;
        }
        
        
        if (followDetection != null)
        {
            foreach (PoseBoundingBox poseBoundingBox in detections)
            {
                if(poseBoundingBox.Class.Id == 1)
                {
                    poseBoundingBox.Class.Id = -1;
                }
            }
            followDetection.Class.Id = 1;
            followTracker.Correct(followDetection);
        }
        else
        {
            followTracker = null;
        }
        
        UpdateTrackIds(frameNumber);
    }

    /// <summary>
    /// Inverse over Union to find overlap between prediction and observation.
    /// </summary>
    static PoseBoundingBox? FindBestBoxFit(
        List<PoseBoundingBox> detections,
        Rectangle tracker,
        float iouThreshold)
    {
        PoseBoundingBox? bestDetection = null;
        float highestIou = iouThreshold;
        float errorCheckIou = 0;
        foreach (PoseBoundingBox detection in detections)
        {
            // find smallest intersection box
            int xx1 = Math.Max(detection.Bounds.Left, tracker.Left);
            int yy1 = Math.Max(detection.Bounds.Top, tracker.Top);
            int xx2 = Math.Min(detection.Bounds.Right, tracker.Right);
            int yy2 = Math.Min(detection.Bounds.Bottom, tracker.Bottom);
            int w = Math.Max(0, xx2 - xx1);
            int h = Math.Max(0, yy2 - yy1);
            int intersection = w * h;
            int detArea = detection.Bounds.Width * detection.Bounds.Height;
            int trkArea = tracker.Width * tracker.Height;
            int union = detArea + trkArea - intersection;
            float iou = intersection /(float) union;
            if (iou > highestIou)
            {
                highestIou = iou;
                bestDetection = detection;
            }

            if (iou > errorCheckIou)
            {
                errorCheckIou = iou;
            }
        }

        if (bestDetection == null)
        {
            Console.WriteLine($"Detection threshold failed. Highest detection: {errorCheckIou}");
        }

        return bestDetection;
    }
    
    #endregion

    #region REFERENCE

    public bool HasPoseAtFrame(int frameNumber, bool isLead)
    {
        if (isLead && LeadPose(frameNumber) == null)
        {
            return false;
        }

        if (!isLead && FollowPose(frameNumber) == null)
        {
            return false;
        }

        return true;
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
        if (isLead)
        {
            PoseBoundingBox? leadPose = LeadPose(frameNumber);
            if (leadPose == null) return 0;
            return leadPose.Keypoints[jointNumber].Confidence;
        }

        PoseBoundingBox? followPose = FollowPose(frameNumber);
        if (followPose == null) return 0;
        return followPose.Keypoints[jointNumber].Confidence;
    }

    public void CopyRotationToNextFrame(int frameNumber)
    {
        // interpolate the frame
        int lastSampleFrame = SampleFrame(frameNumber - 1);
        Vector3 lastAffine = affineTransforms[lastSampleFrame];
        int sampleFrame = SampleFrame(frameNumber);
            
        for (int i = lastSampleFrame + 1; i < sampleFrame; i++)
        {
            lastAffine += affineTransforms[i];
        }

        float pitchAlpha = MathF.Atan2(lastAffine.Y * PixelToMeter, focalLength);
        float yawAlpha = MathF.Atan2(lastAffine.X * PixelToMeter, focalLength);

        rotationsPerFrame[frameNumber] = Quaternion.CreateFromAxisAngle(Right(frameNumber - 1), -pitchAlpha) *
                                         Quaternion.CreateFromAxisAngle(Up(frameNumber - 1), -yawAlpha) *
                                         //Quaternion.CreateFromAxisAngle(Forward(frameNumber - 1), lastAffine.Z) * // roll
                                         rotationsPerFrame[frameNumber - 1];
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

    int SampleFrame(int frameNumber)
    {
        return (int)Math.Round(startingFrame + frameNumber * (maxFrame / (float)totalFrameCountAt30Fps));
    }
    
    void UpdateTrackIds(int frameNumber)
    {
        using SQLiteConnection connection = new($"URI=file:{dbPath}");
        connection.Open();

        using SQLiteTransaction? transaction = connection.BeginTransaction();
        foreach (PoseBoundingBox entry in allPosesAndConfidencesPerFrame[frameNumber])
        {
            SQLiteCommand? command = connection.CreateCommand();
            command.CommandText = $"UPDATE table_{FilePrefix()} SET track_id = @trackId WHERE id = @rowId";
            command.Parameters.AddWithValue("@trackId", entry.Class.Id);
            command.Parameters.AddWithValue("@rowId", entry.DbId);

            command.ExecuteNonQuery();
        }

        transaction.Commit();

        connection.Close();
    }

    #endregion

    #region GETTERS

    public Tuple<PoseBoundingBox?, PoseBoundingBox?> GetLeadAndFollowPoseForFrame(int frameNumber)
    {
        return new Tuple<PoseBoundingBox?, PoseBoundingBox?>(LeadPose(frameNumber), FollowPose(frameNumber));
    }

    /// <summary>
    /// used for drawing
    /// </summary>
    public List<PoseBoundingBox> GetPosesPerDancerAtFrame(int frameNumber)
    {
        return allPosesAndConfidencesPerFrame[frameNumber];
    }
    
    PoseBoundingBox? LeadPose(int frameNumber) => allPosesAndConfidencesPerFrame[frameNumber]
        .Find(x => x.Class.Id == 0);
    
    PoseBoundingBox? FollowPose(int frameNumber) => allPosesAndConfidencesPerFrame[frameNumber]
        .Find(x => x.Class.Id == 1);

    string FilePrefix()
    {
        return Path.GetFileNameWithoutExtension(name).Split("-")[0];
    }

    #endregion
}