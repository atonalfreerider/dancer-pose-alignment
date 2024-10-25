using System.Numerics;
using Newtonsoft.Json;

namespace dancer_pose_alignment;

[Serializable]
public class PoseBoundingBox
{
    public int id;
    public List<float> bbox;
    public float confidence;
    public List<List<float>> keypoints;
    
    [JsonIgnore]public List<Keypoint> Keypoints => keypoints.Select(x => new Keypoint(x[0], x[1], x[2])).ToList();
    
    [JsonIgnore] public List<Vector3> RecenteredKeypoints;
    [JsonIgnore] public Rectangle Bounds => new Rectangle(bbox[0], bbox[1], bbox[2], bbox[3]);
    [JsonIgnore] public Class Class;

}

public class Keypoint(float x, float y, float confidence)
{
    public Point Point = new((int)Math.Round(x), (int)Math.Round(y));
    public float Confidence = confidence;
}

public class Point(int x, int y)
{
    public readonly int X = x;
    public readonly int Y = y;
    
}

public class Class(int id, string name)
{
    public int Id = id;
    public string Name = name;
}

public class Rectangle
{
    public int X;
    public int Y;
    public int Width;
    public int Height;
    public bool IsEmpty;
    public Point Location;
    public int Left;
    public int Top;
    public int Right;
    public int Bottom;

    public Rectangle(float x1, float y1, float x2, float y2)
    {
        // Set the corners of the rectangle
        Left = (int)x1;
        Top = (int)y1;
        Right = (int)x2;
        Bottom = (int)y2;

        // Calculate X, Y (top-left corner), width, and height
        X = Left;
        Y = Top;
        Width = Math.Abs(Right - Left);
        Height = Math.Abs(Bottom - Top);

        // Set the location as the top-left corner
        Location = new Point(X, Y);

        // A rectangle is empty if it has no width or height
        IsEmpty = Width == 0 || Height == 0;
    }
}

[Serializable]
public class Slam
{
    public float x;
    public float y;
    public float z;
    public float q_x;
    public float q_y;
    public float q_z;
    public float q_w;
}