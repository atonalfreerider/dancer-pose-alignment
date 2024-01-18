namespace dancer_pose_alignment;

public class PoseBoundingBox
{
    public List<Keypoint> Keypoints;
    public Class Class;
    public Rectangle Bounds;
    public float Confidence;
}

public class Keypoint
{
    public int Index;
    public Point Point;
    public float Confidence;
}

public class Point(int x, int y)
{
    public int X = x;
    public int Y = y;
    public bool IsEmpty;
}

public class Class
{
    public int Id;
    public string Name;
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
}