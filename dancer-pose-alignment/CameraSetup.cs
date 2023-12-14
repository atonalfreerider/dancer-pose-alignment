using System.Numerics;

namespace dancer_pose_alignment;

public class CameraSetup(Vector2 size, int totalFrameCount, PoseType poseType)
{
    public readonly Vector3[] PositionsPerFrame = new Vector3[totalFrameCount];
    public readonly Quaternion[] RotationsPerFrame = new Quaternion[totalFrameCount];
    public float FocalLength = .05f;

    const float PixelToMeter = 0.000264583f;

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

    public void SetAllPosesAtFrame(List<List<Vector3>> allPoses, int frameNumber)
    {
        allPosesAndConfidencesPerFrame[frameNumber] = allPoses;
        recenteredRescaledAllPosesPerFrame[frameNumber] = allPoses.Select(pose => pose.Select(vec =>
                new Vector3(
                    (vec.X - size.X / 2) * PixelToMeter,
                    -(vec.Y - size.Y / 2) * PixelToMeter, // flip
                    vec.Z)) // keep the confidence
            .ToList()).ToList();

        leadIndicesPerFrame[frameNumber] = -1;
        followIndicesPerFrame[frameNumber] = -1;
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
    
    List<Vector3> Adjusted(IEnumerable<Vector3> keypoints, int frame)
    {
        // Translate keypoints to the camera center 
        Vector3 cameraCenter = PositionsPerFrame[frame];
        List<Vector3> adjustedKeypoints = keypoints.Select(vec => cameraCenter + vec).ToList();

        // Rotate keypoints around the camera center by the camera's rotation quaternion 
        Quaternion rotation = RotationsPerFrame[frame];
        for (int i = 0; i < adjustedKeypoints.Count; i++)
        {
            adjustedKeypoints[i] = Vector3.Transform(adjustedKeypoints[i] - cameraCenter, rotation) + cameraCenter;
        }

        // Translate keypoints to the camera's focal length 
        return adjustedKeypoints.Select(vec => vec + Forward(frame) * FocalLength).ToList();
    }

    public Vector2 ReverseProjectPoint(Vector3 worldPoint, int frameNumber)
    {
        Vector3 target = TargetAtFrame(worldPoint, frameNumber);
        Vector2 imagePlaneCoordinates = GetImagePlaneCoordinates(target, frameNumber);

        Vector2 offcenterAndRescaleAndFlip = new Vector2(
            imagePlaneCoordinates.X / PixelToMeter + size.X / 2,
            -imagePlaneCoordinates.Y / PixelToMeter + size.Y / 2);

        // don't render off screen
        if (offcenterAndRescaleAndFlip.Y < 0)
        {
            offcenterAndRescaleAndFlip.Y = 0;
        }

        if (offcenterAndRescaleAndFlip.Y > size.Y)
        {
            offcenterAndRescaleAndFlip.Y = size.Y;
        }

        if (offcenterAndRescaleAndFlip.X < 0)
        {
            offcenterAndRescaleAndFlip.X = 0;
        }

        if (offcenterAndRescaleAndFlip.X > size.X)
        {
            offcenterAndRescaleAndFlip.X = size.X;
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

    public float Error(IEnumerable<Vector3> pose3D, bool isLead, int frameNumber)
    {
        List<Vector3> comparePose = isLead 
            ? recenteredRescaledAllPosesPerFrame[frameNumber][leadIndicesPerFrame[frameNumber]]
            : recenteredRescaledAllPosesPerFrame[frameNumber][followIndicesPerFrame[frameNumber]];

        return pose3D.Select(vec => ReverseProjectPoint(vec, frameNumber))
            .Select((target, i) => Vector2.Distance(
                target, 
                new Vector2(comparePose[i].X, comparePose[i].Y)) * comparePose[i].Z).Sum();
    }

    /// <summary>
    /// Provides a normalized vector from this camera to a target in 3D space at a given frame
    /// </summary>
    Vector3 TargetAtFrame(Vector3 vector3, int frameNumber)
    {
        return Vector3.Normalize(vector3 - PositionsPerFrame[frameNumber]);
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

        Vector3 stanceWidth = new Vector3(-.3f, 0f, 0f);
        const float camRadius = 5f;
        const float camHeight = 1.5f;

        // calculate slope and orientation of lead ankle stance, so that iteration can match it 
        float leadPoseAnkleSlope = (leadLeftAnkle.Y - leadRightAnkle.Y) / (leadLeftAnkle.X - leadRightAnkle.X);
        bool isFacingLead = leadRightAnkle.X < leadLeftAnkle.X;
        if (!isFacingLead)
        {
            leadPoseAnkleSlope = (leadRightAnkle.Y - leadLeftAnkle.Y) / (leadRightAnkle.X - leadLeftAnkle.X);
        }

        // rotate camera in circle at 5m radius and 1.5m elevation pointed at origin until orientation and slope matches 
        for (float alpha = 0; alpha < 2 * MathF.PI; alpha += .001f)
        {
            PositionsPerFrame[0] = new Vector3(
                camRadius * MathF.Sin(alpha),
                camHeight,
                camRadius * MathF.Cos(alpha));

            RotationsPerFrame[0] = Transform.LookAt(
                Vector3.Zero,
                Quaternion.Identity,
                PositionsPerFrame[0]);

            Vector2 origin = ReverseProjectPoint(Vector3.Zero, 0);
            Vector2 leadStance = ReverseProjectPoint(stanceWidth, 0); // lead left ankle 

            float slopeOfCamera = (leadStance.Y - origin.Y) / (leadStance.X - origin.X);
            bool isFacingLeadInCamera = origin.X < leadStance.X;
            if (!isFacingLeadInCamera)
            {
                slopeOfCamera = (origin.Y - leadStance.Y) / (origin.X - leadStance.X);
            }

            if (isFacingLead == isFacingLeadInCamera && MathF.Abs(leadPoseAnkleSlope - slopeOfCamera) < .01f)
            {
                break;
            }
        }
        
        CenterRoll(); 
        CenterRightLeadAnkleOnOrigin(leadRightAnkle); 
        CenterRoll(); 
        CenterRightLeadAnkleOnOrigin(leadRightAnkle);
        
        const float hipHeight = .75f; 
        float leadHipY = allPosesAndConfidencesPerFrame[0][leadIndicesPerFrame[0]][JointExtension.RHipIndex(poseType)].Y;
        
        while (leadHipY < ReverseProjectPoint(new Vector3(0, hipHeight, 0), 0).Y) 
        { 
            FocalLength += .001f; 
            CenterRightLeadAnkleOnOrigin(leadRightAnkle);
        }
        
        while (leadHipY > ReverseProjectPoint(new Vector3(0, hipHeight, 0), 0).Y) 
        { 
            FocalLength -= .001f; 
            CenterRightLeadAnkleOnOrigin(leadRightAnkle);
        }
        
        CenterRoll(); 
        CenterRightLeadAnkleOnOrigin(leadRightAnkle); 
        CenterRoll(); 
        CenterRightLeadAnkleOnOrigin(leadRightAnkle); 

    }

    void CenterRightLeadAnkleOnOrigin(Vector2 leadRightAnkle)
    {
        // yaw and pitch the camera until the origin is centered at the lead right ankle
        Vector2 origin = ReverseProjectPoint(Vector3.Zero, 0);
        while (leadRightAnkle.X < origin.X)
        {
            RotationsPerFrame[0] *= Quaternion.CreateFromAxisAngle(Vector3.UnitY, .001f);
            origin = ReverseProjectPoint(Vector3.Zero, 0);
        }

        while (leadRightAnkle.X > origin.X)
        {
            RotationsPerFrame[0] *= Quaternion.CreateFromAxisAngle(Vector3.UnitY, -.001f);
            origin = ReverseProjectPoint(Vector3.Zero, 0);
        }

        while (leadRightAnkle.Y > origin.Y)
        {
            // pitch up 
            RotationsPerFrame[0] *= Quaternion.CreateFromAxisAngle(Vector3.UnitX, -.001f);
            origin = ReverseProjectPoint(Vector3.Zero, 0);
        }

        while (leadRightAnkle.Y < origin.Y)
        {
            // pitch down 
            RotationsPerFrame[0] *= Quaternion.CreateFromAxisAngle(Vector3.UnitX, .001f);
            origin = ReverseProjectPoint(Vector3.Zero, 0);
        }
    }

    void CenterRoll()
    {
        Vector2 unitY = ReverseProjectPoint(Vector3.UnitY, 0);
        Vector2 origin = ReverseProjectPoint(Vector3.Zero, 0);

        while (unitY.X < origin.X)
        {
            // roll left 
            RotationsPerFrame[0] *= Quaternion.CreateFromAxisAngle(Vector3.UnitZ, .001f);
            origin = ReverseProjectPoint(Vector3.Zero, 0);
            unitY = ReverseProjectPoint(Vector3.UnitY, 0);
        }

        while (unitY.X > origin.X)
        {
            // roll right 
            RotationsPerFrame[0] *= Quaternion.CreateFromAxisAngle(Vector3.UnitZ, -.001f);
            origin = ReverseProjectPoint(Vector3.Zero, 0);
            unitY = ReverseProjectPoint(Vector3.UnitY, 0);
        }
    }

    /// <summary>
    /// Should only be called on frame 0
    /// </summary>
    public bool IterateHeight(CameraPoseSolver poseSolver)
    {
        const float delta = 0.5f;
        if (PositionsPerFrame[0].Y + delta > 2.5 || PositionsPerFrame[0].Y - delta < 0.05f) return false;
        
        float currentError = poseSolver.Calculate3DPosesAndTotalError();
        
        Vector2 leadRightAnkle = new Vector2(
            allPosesAndConfidencesPerFrame[0][leadIndicesPerFrame[0]][JointExtension.RAnkleIndex(poseType)].X,
            allPosesAndConfidencesPerFrame[0][leadIndicesPerFrame[0]][JointExtension.RAnkleIndex(poseType)].Y);

        // yaw camera left and right
        float originalHeight = PositionsPerFrame[0].Y;
        Quaternion originalRotation = RotationsPerFrame[0];

        
        
        PositionsPerFrame[0] = new Vector3(
            PositionsPerFrame[0].X,
            PositionsPerFrame[0].Y + delta,
            PositionsPerFrame[0].Z);
        
        CenterRightLeadAnkleOnOrigin(leadRightAnkle);
        float upError = poseSolver.Calculate3DPosesAndTotalError();
        
        PositionsPerFrame[0] = new Vector3(
            PositionsPerFrame[0].X,
            originalHeight -  delta,
            PositionsPerFrame[0].Z);
        
        CenterRightLeadAnkleOnOrigin(leadRightAnkle);
        float downError = poseSolver.Calculate3DPosesAndTotalError();

        if (upError < downError && upError < currentError)
        {
            PositionsPerFrame[0] = new Vector3(
                PositionsPerFrame[0].X,
                originalHeight + delta,
                PositionsPerFrame[0].Z);
            CenterRightLeadAnkleOnOrigin(leadRightAnkle);
            return true;
        }

        if (downError < upError && downError < currentError)
        {
            return true;
        }

        // reset
        PositionsPerFrame[0] = new Vector3(
            PositionsPerFrame[0].X,
            originalHeight,
            PositionsPerFrame[0].Z);
        RotationsPerFrame[0] = originalRotation;
        return false;
    }

    public bool IterateOrientation(CameraPoseSolver poseSolver, int frameNumber)
    {
        bool yawed = IterateYaw(0.01f, poseSolver, frameNumber);
        bool pitched = IteratePitch(0.01f, poseSolver, frameNumber);
        bool rolled = IterateRoll(0.01f, poseSolver, frameNumber);

        return yawed || pitched || rolled;
    }

    bool IterateYaw(float yawStepSize, CameraPoseSolver poseSolver, int frameNumber)
    {
        float currentError = poseSolver.Calculate3DPosesAndTotalError();

        // yaw camera left and right
        Quaternion originalRotation = RotationsPerFrame[frameNumber];

        Quaternion leftYawRotation = originalRotation *
                                     Quaternion.CreateFromAxisAngle(Vector3.UnitY, yawStepSize);
        Quaternion rightYawRotation = originalRotation *
                                      Quaternion.CreateFromAxisAngle(Vector3.UnitY, -yawStepSize);

        RotationsPerFrame[frameNumber] = leftYawRotation;
        float leftYawError = poseSolver.Calculate3DPosesAndTotalError();

        RotationsPerFrame[frameNumber] = rightYawRotation;
        float rightYawError = poseSolver.Calculate3DPosesAndTotalError();

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

    bool IteratePitch(float pitchStepSize, CameraPoseSolver poseSolver, int frameNumber)
    {
        float currentError = poseSolver.Calculate3DPosesAndTotalError();

        // pitch camera up and down
        Quaternion originalRotation = RotationsPerFrame[frameNumber];
        Quaternion upPitchRotation = originalRotation *
                                     Quaternion.CreateFromAxisAngle(Vector3.UnitX, -pitchStepSize);

        // check if the angle between the up pitch forward vector and the ground forward vector is greater than 15 degrees
        Vector3 upPitchForward = Vector3.Normalize(Vector3.Transform(Vector3.UnitZ, upPitchRotation));

        float angleBetween = MathF.Acos(Vector3.Dot(upPitchForward, Forward(frameNumber)));
        if (angleBetween > MathF.PI / 12)
        {
            Console.WriteLine("pitch angle too high" + angleBetween);
            RotationsPerFrame[frameNumber] = originalRotation;
        }

        Quaternion downPitchRotation = originalRotation *
                                       Quaternion.CreateFromAxisAngle(Vector3.UnitZ, pitchStepSize);

        // check if the angle between the down pitch forward vector and the ground forward vector is greater than 15 degrees
        Vector3 downPitchForward = Vector3.Normalize(Vector3.Transform(Vector3.UnitZ, downPitchRotation));
        angleBetween = MathF.Acos(Vector3.Dot(downPitchForward, Forward(frameNumber)));
        if (angleBetween > MathF.PI / 12)
        {
            Console.WriteLine("pitch angle too low" + angleBetween);
            RotationsPerFrame[frameNumber] = originalRotation;
        }

        RotationsPerFrame[frameNumber] = upPitchRotation;
        float upPitchError = poseSolver.Calculate3DPosesAndTotalError();
        if (poseSolver.AnyPointsHigherThan2pt5Meters() || poseSolver.LowestLeadAnkleIsMoreThan10CMAboveZero())
        {
            Console.WriteLine("up pitch too high");
            //upPitchError = float.MaxValue;
        }

        RotationsPerFrame[frameNumber] = downPitchRotation;
        float downPitchError = poseSolver.Calculate3DPosesAndTotalError();
        if (poseSolver.AnyPointsBelowGround())
        {
            Console.WriteLine("down pitch too low - figure below ground");
            //downPitchError = float.MaxValue;
        }

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

    bool IterateRoll(float rollStepSize, CameraPoseSolver poseSolver, int frameNumber)
    {
        float currentError = poseSolver.Calculate3DPosesAndTotalError();
        // roll camera left and right
        Quaternion originalRotation = RotationsPerFrame[frameNumber];

        Quaternion leftRollRotation = originalRotation *
                                      Quaternion.CreateFromAxisAngle(Vector3.UnitZ, rollStepSize);

        // check if the up vector has rolled too far to the left relative to the ground up vector
        Vector3 leftRollUp = Vector3.Normalize(Vector3.Transform(Vector3.UnitY, leftRollRotation));

        float angleBetween = MathF.Acos(Vector3.Dot(leftRollUp, Up(frameNumber)));
        if (angleBetween > MathF.PI / 12)
        {
            Console.WriteLine("roll angle too far to the left" + angleBetween);
            RotationsPerFrame[frameNumber] = originalRotation;
        }

        Quaternion rightRollRotation = originalRotation *
                                       Quaternion.CreateFromAxisAngle(Vector3.UnitZ, -rollStepSize);

        // check if the up vector has rolled too far to the right relative to the ground up vector
        Vector3 rightRollUp = Vector3.Transform(Vector3.UnitY, rightRollRotation);
        angleBetween = MathF.Acos(Vector3.Dot(rightRollUp, Up(frameNumber)));
        if (angleBetween > MathF.PI / 12)
        {
            Console.WriteLine("roll angle too far to the right" + angleBetween);
            RotationsPerFrame[frameNumber] = originalRotation;
        }

        RotationsPerFrame[frameNumber] = leftRollRotation;
        float leftRollError = poseSolver.Calculate3DPosesAndTotalError();

        RotationsPerFrame[frameNumber] = rightRollRotation;
        float rightRollError = poseSolver.Calculate3DPosesAndTotalError();

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
            PositionsPerFrame[frameNumber],
            Vector3.Normalize(isLead
                ? leadProjectionsPerFrame[frameNumber][jointNumber] - PositionsPerFrame[frameNumber]
                : followProjectionsPerFrame[frameNumber][jointNumber] - PositionsPerFrame[frameNumber]));
        return rayToJoint;
    }

    public float JointConfidence(int frameNumber, int jointNumber, bool isLead)
    {
        return isLead
            ? recenteredRescaledAllPosesPerFrame[frameNumber][leadIndicesPerFrame[frameNumber]][jointNumber].Z
            : recenteredRescaledAllPosesPerFrame[frameNumber][followIndicesPerFrame[frameNumber]][jointNumber].Z;
    }

    public void CopyPositionsToNextFrame(int frameNumber)
    {
        if (PositionsPerFrame.Length <= frameNumber + 1)
        {
            PositionsPerFrame[frameNumber] = PositionsPerFrame[frameNumber - 1];
        }

        if (RotationsPerFrame.Length <= frameNumber + 1)
        {
            RotationsPerFrame[frameNumber] = RotationsPerFrame[frameNumber - 1];
        }
    }
    
    #endregion
}