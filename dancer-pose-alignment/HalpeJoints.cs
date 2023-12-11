namespace dancer_pose_alignment;

/// <summary>
/// https://github.com/Fang-Haoshu/Halpe-FullBody
/// </summary>
public enum HalpeJoints
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

public static class HalpeExtension
{
    public static bool IsRightSide(HalpeJoints jointIdx)
    {
        return jointIdx is HalpeJoints.RAnkle or HalpeJoints.RBigToe or HalpeJoints.REar or HalpeJoints.RElbow or HalpeJoints.REye or HalpeJoints.RHeel
            or HalpeJoints.RHip or HalpeJoints.RKnee or HalpeJoints.RShoulder or HalpeJoints.RSmallToe or HalpeJoints.RWrist;
    }
}
