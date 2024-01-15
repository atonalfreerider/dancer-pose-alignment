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

public class Point
{
    public Point(float x, float y)
    {
        X = x;
        Y = y;
    }
    
    public float X;
    public float Y;
    public bool IsEmpty;
}

public class Class
{
    public int Id;
    public string Name;
}

public class Rectangle
{
    public float X;
    public float Y;
    public float Width;
    public float Height;
    public bool IsEmpty;
    public Point Location;
    public float Left;
    public float Top;
    public float Right;
    public float Bottom;
}