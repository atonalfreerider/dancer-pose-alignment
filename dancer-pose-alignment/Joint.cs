namespace dancer_pose_alignment;

public enum PoseType
{
    Coco = 0,
    Halpe = 1
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
        _ => -1
    };

    public static int RAnkleIndex(PoseType poseType) => poseType switch
    {
        PoseType.Coco => (int)CocoJoint.R_Ankle,
        PoseType.Halpe => (int)HalpeJoint.RAnkle,
        _ => -1
    };

    public static int RHipIndex(PoseType poseType) => poseType switch
    {
        PoseType.Coco => (int)CocoJoint.R_Hip,
        PoseType.Halpe => (int)HalpeJoint.RHip,
        _ => -1
    };

    public static int PoseCount(PoseType poseType) => poseType == PoseType.Coco
        ? Enum.GetNames<CocoJoint>().Length
        : 26; // Halpe
}