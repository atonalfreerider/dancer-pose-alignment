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
    
    public static int LKneeIndex(PoseType poseType) => poseType switch
    {
        PoseType.Coco => (int)CocoJoint.L_Knee,
        PoseType.Halpe => (int)HalpeJoint.LKnee,
        _ => -1
    };
    
    public static int RKneeIndex(PoseType poseType) => poseType switch
    {
        PoseType.Coco => (int)CocoJoint.R_Knee,
        PoseType.Halpe => (int)HalpeJoint.RKnee,
        _ => -1
    };

    public static int RHipIndex(PoseType poseType) => poseType switch
    {
        PoseType.Coco => (int)CocoJoint.R_Hip,
        PoseType.Halpe => (int)HalpeJoint.RHip,
        _ => -1
    };
    
    public static int LHipIndex(PoseType poseType) => poseType switch
    {
        PoseType.Coco => (int)CocoJoint.L_Hip,
        PoseType.Halpe => (int)HalpeJoint.LHip,
        _ => -1
    };
    
    public static int RShoulderIndex(PoseType poseType) => poseType switch
    {
        PoseType.Coco => (int)CocoJoint.R_Shoulder,
        PoseType.Halpe => (int)HalpeJoint.RShoulder,
        _ => -1
    };
    
    public static int LShoulderIndex(PoseType poseType) => poseType switch
    {
        PoseType.Coco => (int)CocoJoint.L_Shoulder,
        PoseType.Halpe => (int)HalpeJoint.LShoulder,
        _ => -1
    };
    
    public static int RElbowIndex(PoseType poseType) => poseType switch
    {
        PoseType.Coco => (int)CocoJoint.R_Elbow,
        PoseType.Halpe => (int)HalpeJoint.RElbow,
        _ => -1
    };
    
    public static int LElbowIndex(PoseType poseType) => poseType switch
    {
        PoseType.Coco => (int)CocoJoint.L_Elbow,
        PoseType.Halpe => (int)HalpeJoint.LElbow,
        _ => -1
    };
    
    public static int RWristIndex(PoseType poseType) => poseType switch
    {
        PoseType.Coco => (int)CocoJoint.R_Wrist,
        PoseType.Halpe => (int)HalpeJoint.RWrist,
        _ => -1
    };
    
    public static int LWristIndex(PoseType poseType) => poseType switch
    {
        PoseType.Coco => (int)CocoJoint.L_Wrist,
        PoseType.Halpe => (int)HalpeJoint.LWrist,
        _ => -1
    };
    
    public static int REyeIndex(PoseType poseType) => poseType switch
    {
        PoseType.Coco => (int)CocoJoint.R_Eye,
        PoseType.Halpe => (int)HalpeJoint.REye,
        _ => -1
    };
    
    public static int LEyeIndex(PoseType poseType) => poseType switch
    {
        PoseType.Coco => (int)CocoJoint.L_Eye,
        PoseType.Halpe => (int)HalpeJoint.LEye,
        _ => -1
    };
    
    public static int REarIndex(PoseType poseType) => poseType switch
    {
        PoseType.Coco => (int)CocoJoint.R_Ear,
        PoseType.Halpe => (int)HalpeJoint.REar,
        _ => -1
    };
    
    public static int LEarIndex(PoseType poseType) => poseType switch
    {
        PoseType.Coco => (int)CocoJoint.L_Ear,
        PoseType.Halpe => (int)HalpeJoint.LEar,
        _ => -1
    };
    
    public static int NoseIndex(PoseType poseType) => poseType switch
    {
        PoseType.Coco => (int)CocoJoint.Nose,
        PoseType.Halpe => (int)HalpeJoint.Nose,
        _ => -1
    };

    public static int PoseCount(PoseType poseType) => poseType == PoseType.Coco
        ? Enum.GetNames<CocoJoint>().Length
        : 26; // Halpe
}