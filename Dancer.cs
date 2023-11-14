using SixLabors.ImageSharp;

namespace dancer_pose_alignment;

public enum Role
{
    Lead = 0,
    Follow = 1
}

public class Dancer
{
    public readonly List<List<Point>?> PosesByFrame = new();
    public Role Role;

    public Dancer(Role role)
    {
        Role = role;
    }
}