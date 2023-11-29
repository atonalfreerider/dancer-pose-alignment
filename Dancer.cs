using SixLabors.ImageSharp;

namespace dancer_pose_alignment;

public enum Role
{
    Lead = 0,
    Follow = 1,
    Unknown = 2
}

public class Dancer
{
    readonly Dictionary<int, List<Point>?> PosesByFrame = new();
    readonly Dictionary<int, Rectangle?> BoundingBoxesByFrame = new();
    public Role Role;

    public Dancer(Role role)
    {
        Role = role;
    }

    public float AverageHeightForAllPoses()
    {
        float totalHeight = 0;
        foreach (List<Point>? points in PosesByFrame.Values)
        {
            if (points == null) continue;
            totalHeight += points.Max(x => x.Y) - points.Min(x => x.Y);
        }

        return totalHeight / PosesByFrame.Values.Count(x => x != null);
    }

    public bool TryAddPose(int frame, List<Point>? pose, Rectangle? boundingBox)
    {
        return PosesByFrame.TryAdd(frame, pose) && BoundingBoxesByFrame.TryAdd(frame, boundingBox);
    }

    public List<Point>? GetPoseAt(int frame)
    {
        return PosesByFrame[frame];
    }

    public int PoseCount => PosesByFrame.Count;

    public int NonOverlappingAreaWithLastBox(Rectangle box, int frame)
    {
        Rectangle? compare = BoundingBoxesByFrame[LastNotNullPoseIndex(frame)];
        if (compare == null)
        {
            return box.Width * box.Height;
        }

        return CalculateNonOverlappingArea(box, compare.Value);
    }
    
    static int CalculateNonOverlappingArea(Rectangle a, Rectangle b) 
    {
        // Calculate the area of both rectangles
        int area1 = a.Width * a.Height;
        int area2 = b.Width * b.Height;

        // Calculate the overlapping area
        int overlapWidth = Math.Max(0, Math.Min(a.Right, b.Right) - Math.Max(a.Left, b.Left));
        int overlapHeight = Math.Max(0, Math.Min(a.Bottom, b.Bottom) - Math.Max(a.Top, b.Top));
        int overlapArea = overlapWidth * overlapHeight;

        // Calculate the total non-overlapping area
        return area1 + area2 - overlapArea;
    }

    int LastNotNullPoseIndex(int frame)
    {
        while (!PosesByFrame.ContainsKey(frame))
        {
            frame--;
        }

        while (PosesByFrame[frame] == null)
        {
            frame--;

            if (frame < 0) return -1;
        }

        return frame;
    }

    int NextNotNullPoseIndex(int frame)
    {
        if (frame >= PosesByFrame.Count)
        {
            return frame;
        }

        while (PosesByFrame[frame] == null)
        {
            frame++;
            if (frame >= PosesByFrame.Count)
            {
                return frame;
            }
        }

        return frame;
    }

    public void InterpolateMissing()
    {
        for (int i = 0; i < PosesByFrame.Count; i++)
        {
            if (PosesByFrame[i] == null)
            {
                int last = LastNotNullPoseIndex(i);
                int next = NextNotNullPoseIndex(i);

                if (last < 0 || next >= PosesByFrame.Count)
                {
                    return;
                }

                List<Point> lastPose = PosesByFrame[last]!;
                List<Point> nextPose = PosesByFrame[next]!;

                PosesByFrame[i] = InterpolatePoints(lastPose, nextPose, (double)(i - last) / (next - last));
            }
        }
    }

    static List<Point> InterpolatePoints(IReadOnlyList<Point> listA, IReadOnlyList<Point> listB, double fraction)
    {
        List<Point> interpolatedPoints = new List<Point>();
        int count = Math.Min(listA.Count, listB.Count);

        for (int i = 0; i < count; i++)
        {
            int interpolatedX = (int)Math.Round(listA[i].X + (listB[i].X - listA[i].X) * fraction);
            int interpolatedY = (int)Math.Round(listA[i].Y + (listB[i].Y - listA[i].Y) * fraction);
            interpolatedPoints.Add(new Point(interpolatedX, interpolatedY));
        }

        return interpolatedPoints;
    }
}