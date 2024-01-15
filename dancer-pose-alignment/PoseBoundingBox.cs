using Compunet.YoloV8.Data;
using Compunet.YoloV8.Metadata;
using SixLabors.ImageSharp;

namespace dancer_pose_alignment;

public class PoseBoundingBox : IPoseBoundingBox
{
    public YoloV8Class Class { get; }
    public Rectangle Bounds { get; }
    public float Confidence { get; }
    public IKeypoint? GetKeypoint(int id)
    {
        return Keypoints[id];
    }

    public IReadOnlyList<IKeypoint> Keypoints { get; }
}