using System.Numerics;

namespace dancer_pose_alignment;

public class CameraSetup(
    Vector2 size,
    int totalFrameCountAt30Fps,
    int startingFrame,
    int maxFrame,
    List<Dictionary<int, PoseBoundingBox>> leadFollow2D,
    List<List<List<Vector3>>> leadFollow3D)
{
    // camera cylindrical position and intrinsic
    float radius = 3.5f;
    float height = 1.2f;
    float alpha = 0;
    float focalLength = .05f;

    // cartesian position is constant (for now), rotation is tracked per frame
    public Vector3 Position => new(radius * MathF.Sin(alpha), height, radius * MathF.Cos(alpha));
    readonly Quaternion[] rotationsPerFrame = new Quaternion[totalFrameCountAt30Fps];

    // constants
    const float PixelToMeter = 0.000264583f;

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

    /// <summary> 
    /// Called when poses are calculated for every frame 
    /// </summary> 
    public void SetAllPosesAtFrame(int frameNumber)
    {
        if (allPosesAndConfidencesPerFrame[frameNumber] == null ||
            allPosesAndConfidencesPerFrame[frameNumber].Count == 0)
        {
            int sampleFrame = SampleFrame(frameNumber);
            List<PoseBoundingBox> posesAtFrame = leadFollow2D
                .Select(poseList => poseList[sampleFrame])
                .ToList();

            foreach (PoseBoundingBox poseBoundingBox in posesAtFrame)
            {
                poseBoundingBox.RecenteredKeypoints = poseBoundingBox.Keypoints.Select(keypoint => new Vector3(
                    (keypoint.Point.X - size.X / 2) * PixelToMeter,
                    -(keypoint.Point.Y - size.Y / 2) * PixelToMeter, // flip 
                    keypoint.Confidence)).ToList(); // keep the confidence; 
            }

            allPosesAndConfidencesPerFrame[frameNumber] = posesAtFrame;
        }

        if (frameNumber == 0)
        {
            rotationsPerFrame[0] = Quaternion.Identity;
            SetFrameZero();
        }
    }

    /// <summary>
    /// From GVHMR, the camera is placed somewhere on the positive Z axis, and the pose is placed at its own origin
    /// Because the origin may not be where the pose actually is, and we are merging more than one pose world between
    /// two figures, this algorithm moves the poses to match their original 2D position, in addition to zoom/tilting the
    /// camera until everything is in the right place
    /// </summary>
    void SetFrameZero()
    {
        // zoom in/out until lead height 3D matches to 2D
        float leadHeight = leadFollow2D[0][0].Bounds.Height;
        while (leadHeight > ReverseProjectPoint(leadFollow3D[0][0].MinBy(x => x.Y), 0, true).Y -
               ReverseProjectPoint(leadFollow3D[0][0].MaxBy(x => x.Y), 0, true).Y)
        {
            focalLength += .01f;
            if (focalLength > 1) break;
        }

        while (leadHeight <  ReverseProjectPoint(leadFollow3D[0][0].MinBy(x => x.Y), 0, true).Y -
               ReverseProjectPoint(leadFollow3D[0][0].MaxBy(x => x.Y), 0, true).Y)
        {
            focalLength -= .01f;
            if (focalLength <= 0)
            {
                focalLength = .000001f;
                break;
            }
        }
        
        // translate both lead and follow until ankles are zeroed out between 3D and 2D
        for (int i = 0; i < 2; i++)
        {
            Vector3 rightAnkle3dOrig = leadFollow3D[i][0][(int)SmplJoint.R_Ankle];
            Vector3 rightAnkle3d =
                new Vector3(rightAnkle3dOrig.X, rightAnkle3dOrig.Y, rightAnkle3dOrig.Z);
            float rightAnkle2Dx = leadFollow2D[i][0].Keypoints[(int)CocoJoint.R_Ankle].Point.X;

            float translation = 0;
            while (rightAnkle2Dx < ReverseProjectPoint(rightAnkle3d, 0, true).X)
            {
                rightAnkle3d.X += PixelToMeter;
                translation += PixelToMeter;
                if (translation > 10) break;
            }

            while (rightAnkle2Dx > ReverseProjectPoint(rightAnkle3d, 0, true).X)
            {
                rightAnkle3d.X -= PixelToMeter;
                translation -= PixelToMeter;

                if (translation < -10) break;
            }

            leadFollow3D[i] = leadFollow3D[i]
                .Select(frame => frame.Select(vector3 => vector3 with { X = vector3.X + translation })
                    .ToList())
                .ToList();
        }

        // camera tilt for lead until r ankle is zeroed out for 3d to 2d
        Vector3 updatedLeadRightAnkle = leadFollow3D[0][0][(int)SmplJoint.R_Ankle];
        float leadRightAnkle2Dy = leadFollow2D[0][0].Keypoints[(int)CocoJoint.R_Ankle].Point.Y;
        int count = 0;
        while (leadRightAnkle2Dy > ReverseProjectPoint(updatedLeadRightAnkle, 0, true).Y)
        {
            rotationsPerFrame[0] *= Quaternion.CreateFromAxisAngle(Vector3.UnitX, .02f);
            count++;
            if (count > 100) break;
        }

        while (leadRightAnkle2Dy < ReverseProjectPoint(updatedLeadRightAnkle, 0, true).Y)
        {
            rotationsPerFrame[0] *= Quaternion.CreateFromAxisAngle(Vector3.UnitX, -.02f);
            count++;
            if (count > 100) break;
        }
    }

    public void SetAllSlam(Dictionary<int, Slam> slam)
    {
        for (int i = 1; i < slam.Count; i++)
        {
            int sampleFrame = SampleFrame(i);
            Quaternion rot = new Quaternion(
                slam[sampleFrame].q_x,
                slam[sampleFrame].q_y,
                slam[sampleFrame].q_z,
                slam[sampleFrame].q_w);
            rotationsPerFrame[i] = rotationsPerFrame[0] * rot;
        }
    }

    #region USER MARKUP

    public Tuple<PoseBoundingBox, int> MarkDancer(Vector2 click, int frameNumber, string selectedButton)
    {
        (PoseBoundingBox closestPose, int jointSelected) = GetClosestIndexAndJointSelected(click, frameNumber);

        switch (selectedButton)
        {
            case "Lead":
                if (closestPose.Class.Id == 0)
                {
                    // deselect
                    closestPose.Class.Id = -1;
                    //UpdateTrackIds(frameNumber);
                    break;
                }

                foreach (PoseBoundingBox poseBoundingBox in allPosesAndConfidencesPerFrame[frameNumber])
                {
                    if (poseBoundingBox.Class.Id == 0)
                    {
                        poseBoundingBox.Class.Id = -1;
                    }
                }

                closestPose.Class.Id = 0;
                //UpdateTrackIds(frameNumber);
                break;
            case "Follow":
                if (closestPose.Class.Id == 1)
                {
                    // deselect
                    closestPose.Class.Id = -1;
                    //UpdateTrackIds(frameNumber);
                    break;
                }

                foreach (PoseBoundingBox poseBoundingBox in allPosesAndConfidencesPerFrame[frameNumber])
                {
                    if (poseBoundingBox.Class.Id == 1)
                    {
                        poseBoundingBox.Class.Id = -1;
                    }
                }

                closestPose.Class.Id = 1;
                //UpdateTrackIds(frameNumber);
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

    #endregion

    #region PROJECTION

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
            imagePlaneCoordinates.Y / PixelToMeter + size.Y / 2);

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

    public List<Vector2> ReverseProjectPose3D(bool isLead, int frameNumber)
    {
        List<Vector3> poseToReverse = leadFollow3D[isLead ? 0 : 1][frameNumber];
        return poseToReverse.Select(vec => ReverseProjectPoint(vec, frameNumber)).ToList();
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
                ? leadFollow3D[0][frameNumber][jointNumber] - Position
                : leadFollow3D[0][frameNumber][jointNumber] - Position));
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

    Vector3? ImgPtRayFloorIntersection(Vector2 imgPt)
    {
        Vector3 projectedPoint = ProjectPoint(imgPt);
        Ray rayFromImgPoint = new(Position, Vector3.Normalize(projectedPoint - Position));

        return Transform.RayPlaneIntersection(new Plane(Vector3.UnitY, 0), rayFromImgPoint);
    }

    int SampleFrame(int frameNumber)
    {
        return (int)Math.Round(startingFrame + (frameNumber) * (maxFrame / (float)totalFrameCountAt30Fps));
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

    PoseBoundingBox? LeadPose(int frameNumber) => leadFollow2D[0][frameNumber];

    PoseBoundingBox? FollowPose(int frameNumber) => leadFollow2D[1][frameNumber];

    #endregion
}