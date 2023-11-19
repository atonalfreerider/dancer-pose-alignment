using SixLabors.ImageSharp;

namespace dancer_pose_alignment;

public class Frame
{
    public int FrameId;
    public readonly List<List<Point>> Poses = new();
    public readonly List<Rectangle> BoundingBoxes = new();

    public Frame(int frameId)
    {
        FrameId = frameId;
    }
}