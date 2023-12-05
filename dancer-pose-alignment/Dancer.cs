using System.Numerics;

namespace dancer_pose_alignment;

public enum Role
{
    Lead = 0,
    Follow = 1
}

public class Dancer
{
    public readonly List<List<Vector3>> PosesByFrame = new();
    public Role Role;
    public float averageZ;
    public Vector2 lastThoraxPosition;
}