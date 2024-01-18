using System.Numerics;
using Newtonsoft.Json;

namespace dancer_pose_alignment;

[Serializable]
public class PoseBoundingBox
{
    public List<Keypoint> Keypoints;
    public Class Class;
    public Rectangle Bounds;
    public float Confidence;

    [JsonIgnore] public int DbId;

    [JsonIgnore] public List<Vector3> RecenterdKeypoints;
}

[Serializable]
public class Keypoint
{
    public int Index;
    public Point Point;
    public float Confidence;
}

[Serializable]
public class Point(int x, int y)
{
    public readonly int X = x;
    public readonly int Y = y;
    public bool IsEmpty;
}

[Serializable]
public class Class(int id, string name)
{
    public int Id = id;
    public string Name = name;
}

[Serializable]
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
}