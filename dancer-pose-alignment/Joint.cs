namespace dancer_pose_alignment;

public enum PoseType
{
    Coco = 0,
    Halpe = 1,
    Smpl = 2
}

public enum CocoJoint
{
    Nose = 0,
    L_Eye = 1,
    R_Eye = 2,
    L_Ear = 3,
    R_Ear = 4,
    L_Shoulder = 5,
    R_Shoulder = 6,
    L_Elbow = 7,
    R_Elbow = 8,
    L_Wrist = 9,
    R_Wrist = 10,
    L_Hip = 11,
    R_Hip = 12,
    L_Knee = 13,
    R_Knee = 14,
    L_Ankle = 15,
    R_Ankle = 16
}

/// <summary>
/// https://github.com/Fang-Haoshu/Halpe-FullBody
/// </summary>
public enum HalpeJoint
{
    Nose = 0,
    LEye = 1,
    REye = 2,
    LEar = 3,
    REar = 4,
    LShoulder = 5,
    RShoulder = 6,
    LElbow = 7,
    RElbow = 8,
    LWrist = 9,
    RWrist = 10,
    LHip = 11,
    RHip = 12,
    LKnee = 13,
    RKnee = 14,
    LAnkle = 15,
    RAnkle = 16,
    Head = 17,
    Neck = 18,
    Hip = 19,
    LBigToe = 20,
    RBigToe = 21,
    LSmallToe = 22,
    RSmallToe = 23,
    LHeel = 24,
    RHeel = 25,

    // 68 Face Keypoints
    Face1 = 26,
    Face2 = 27,
    Face3 = 28,
    Face4 = 29,

    // ...
    Face68 = 93,

    // 21 Left Hand Keypoints
    LHand1 = 94,
    LHand2 = 95,

    // ...
    LHand21 = 114,

    // 21 Right Hand Keypoints
    RHand1 = 115,
    RHand2 = 116,

    // ...
    RHand21 = 135
}

enum SmplJoint
{
    Pelvis = 0,
    L_Hip = 1,
    R_Hip = 2,
    Spine1 = 3,
    L_Knee = 4,
    R_Knee = 5,
    Spine2 = 6,
    L_Ankle = 7,
    R_Ankle = 8,
    Spine3 = 9,
    L_Foot = 10,
    R_Foot = 11,
    Neck = 12,
    L_Collar = 13,
    R_Collar = 14,
    Head = 15,
    L_Shoulder = 16,
    R_Shoulder = 17,
    L_Elbow = 18,
    R_Elbow = 19,
    L_Wrist = 20,
    R_Wrist = 21,
    L_Hand = 22,
    R_Hand = 23
}

public enum CocoLimbs
{
    // Precomputed with Szudzik pairing to correspond with joint indices
    R_Upper_Arm = 70,
    L_Upper_Arm = 54,
    R_Forearm = 108,
    L_Forearm = 88,
    R_Thigh = 208,
    L_Thigh = 180,
    R_Calf = 270,
    L_Calf = 238,
    Pelvis = 167,
    Shoulders = 47
}

public enum SmplLimbs
{
    L_Calf = 60, // L_Ankle to L_Knee
    R_Calf = 77, // R_Ankle to R_Knee
    L_Thigh = 17, // L_Hip to L_Knee
    R_Thigh = 27, // R_Hip to R_Knee
    L_HipToPelvis = 2, // L_Hip to Pelvis
    R_HipToPelvis = 6, // R_Hip to Pelvis
    L_UpperArm = 340, // L_Shoulder to L_Elbow
    R_UpperArm = 378, // R_Shoulder to R_Elbow
    L_Forearm = 418, // L_Elbow to L_Wrist
    R_Forearm = 460, // R_Elbow to R_Wrist
    PelvisToSpine1 = 9, // Pelvis to Spine1
    Spine3ToSpine2 = 96, // Spine3 to Spine2
    Spine2ToSpine1 = 45, // Spine2 to Spine1
    Spine3ToNeck = 153, // Spine3 to Neck
    NeckToHead = 237, // Neck to Head
    L_Foot = 107, // L_Ankle to L_Foot
    R_Foot = 129, // R_Ankle to R_Foot
    L_Hand = 526, // L_Hand to L_Wrist
    R_Hand = 573, // R_Hand to R_Wrist
    L_CollarToShoulder = 285, // L_Shoulder to L_Collar
    R_CollarToShoulder = 320, // R_Shoulder to R_Collar
    L_CollarToNeck = 194, // L_Collar to Neck
    R_CollarToNeck = 222 // R_Collar to Neck
}

public static class JointExtension
{
    public static bool IsRightSide(HalpeJoint halpeJointIdx)
    {
        return halpeJointIdx is HalpeJoint.RAnkle or HalpeJoint.RBigToe or HalpeJoint.REar or HalpeJoint.RElbow
            or HalpeJoint.REye or HalpeJoint.RHeel or HalpeJoint.RHip or HalpeJoint.RKnee or HalpeJoint.RShoulder
            or HalpeJoint.RSmallToe or HalpeJoint.RWrist;
    }

    public static int LAnkleIndex(PoseType poseType) => poseType switch
    {
        PoseType.Coco => (int)CocoJoint.L_Ankle,
        PoseType.Halpe => (int)HalpeJoint.LAnkle,
        PoseType.Smpl => (int)SmplJoint.L_Ankle,
        _ => -1
    };

    public static int RAnkleIndex(PoseType poseType) => poseType switch
    {
        PoseType.Coco => (int)CocoJoint.R_Ankle,
        PoseType.Halpe => (int)HalpeJoint.RAnkle,
        PoseType.Smpl => (int)SmplJoint.R_Ankle,
        _ => -1
    };

    public static int LKneeIndex(PoseType poseType) => poseType switch
    {
        PoseType.Coco => (int)CocoJoint.L_Knee,
        PoseType.Halpe => (int)HalpeJoint.LKnee,
        PoseType.Smpl => (int)SmplJoint.L_Knee,
        _ => -1
    };

    public static int RKneeIndex(PoseType poseType) => poseType switch
    {
        PoseType.Coco => (int)CocoJoint.R_Knee,
        PoseType.Halpe => (int)HalpeJoint.RKnee,
        PoseType.Smpl => (int)SmplJoint.R_Knee,
        _ => -1
    };

    public static int RHipIndex(PoseType poseType) => poseType switch
    {
        PoseType.Coco => (int)CocoJoint.R_Hip,
        PoseType.Halpe => (int)HalpeJoint.RHip,
        PoseType.Smpl => (int)SmplJoint.R_Hip,
        _ => -1
    };

    public static int LHipIndex(PoseType poseType) => poseType switch
    {
        PoseType.Coco => (int)CocoJoint.L_Hip,
        PoseType.Halpe => (int)HalpeJoint.LHip,
        PoseType.Smpl => (int)SmplJoint.L_Hip,
        _ => -1
    };

    public static int RShoulderIndex(PoseType poseType) => poseType switch
    {
        PoseType.Coco => (int)CocoJoint.R_Shoulder,
        PoseType.Halpe => (int)HalpeJoint.RShoulder,
        PoseType.Smpl => (int)SmplJoint.R_Shoulder,
        _ => -1
    };

    public static int LShoulderIndex(PoseType poseType) => poseType switch
    {
        PoseType.Coco => (int)CocoJoint.L_Shoulder,
        PoseType.Halpe => (int)HalpeJoint.LShoulder,
        PoseType.Smpl => (int)SmplJoint.L_Shoulder,
        _ => -1
    };

    public static int RElbowIndex(PoseType poseType) => poseType switch
    {
        PoseType.Coco => (int)CocoJoint.R_Elbow,
        PoseType.Halpe => (int)HalpeJoint.RElbow,
        PoseType.Smpl => (int)SmplJoint.R_Elbow,
        _ => -1
    };

    public static int LElbowIndex(PoseType poseType) => poseType switch
    {
        PoseType.Coco => (int)CocoJoint.L_Elbow,
        PoseType.Halpe => (int)HalpeJoint.LElbow,
        PoseType.Smpl => (int)SmplJoint.L_Elbow,
        _ => -1
    };

    public static int RWristIndex(PoseType poseType) => poseType switch
    {
        PoseType.Coco => (int)CocoJoint.R_Wrist,
        PoseType.Halpe => (int)HalpeJoint.RWrist,
        PoseType.Smpl => (int)SmplJoint.R_Wrist,
        _ => -1
    };

    public static int LWristIndex(PoseType poseType) => poseType switch
    {
        PoseType.Coco => (int)CocoJoint.L_Wrist,
        PoseType.Halpe => (int)HalpeJoint.LWrist,
        PoseType.Smpl => (int)SmplJoint.L_Wrist,
        _ => -1
    };

    public static int REyeIndex(PoseType poseType) => poseType switch
    {
        PoseType.Coco => (int)CocoJoint.R_Eye,
        PoseType.Halpe => (int)HalpeJoint.REye,
        PoseType.Smpl => (int)SmplJoint.Head,
        _ => -1
    };

    public static int LEyeIndex(PoseType poseType) => poseType switch
    {
        PoseType.Coco => (int)CocoJoint.L_Eye,
        PoseType.Halpe => (int)HalpeJoint.LEye,
        PoseType.Smpl => (int)SmplJoint.Head,
        _ => -1
    };

    public static int REarIndex(PoseType poseType) => poseType switch
    {
        PoseType.Coco => (int)CocoJoint.R_Ear,
        PoseType.Halpe => (int)HalpeJoint.REar,
        PoseType.Smpl => (int)SmplJoint.Head,
        _ => -1
    };

    public static int LEarIndex(PoseType poseType) => poseType switch
    {
        PoseType.Coco => (int)CocoJoint.L_Ear,
        PoseType.Halpe => (int)HalpeJoint.LEar,
        PoseType.Smpl => (int)SmplJoint.Head,
        _ => -1
    };

    public static int NoseIndex(PoseType poseType) => poseType switch
    {
        PoseType.Coco => (int)CocoJoint.Nose,
        PoseType.Halpe => (int)HalpeJoint.Nose,
        PoseType.Smpl => (int)SmplJoint.Head,
        _ => -1
    };

    public static int PoseCount(PoseType poseType) => poseType switch
    {
        PoseType.Coco => Enum.GetNames<CocoJoint>().Length,
        PoseType.Halpe => 26,
        PoseType.Smpl => Enum.GetNames<SmplJoint>().Length,
        _ => 0
    };
}

public static class Szudzik
{
    public static ulong uintSzudzik2tupleCombine(uint x, uint y)
    {
        if (x != Math.Max(x, y))
            return (y * y) + x;
        return (x * x) + x + y;
    }

    public static uint[] uintSzudzik2tupleReverse(ulong z) //this number WILL be positive
    {
        uint zSpecial1 = (uint) Math.Floor(Math.Sqrt(z)); //this number WILL be positive (returns integer)
        ulong zSpecial2 = z - (zSpecial1 * zSpecial1); //this number WILL be positive (returns integer)

        return zSpecial2 < zSpecial1 
            ? new[] {(uint) zSpecial2, zSpecial1} 
            : new[] {zSpecial1, (uint) (zSpecial2 - zSpecial1)};
    }
}