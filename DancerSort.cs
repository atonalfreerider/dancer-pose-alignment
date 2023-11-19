using SixLabors.ImageSharp;

namespace dancer_pose_alignment;

public class DancerSort
{
    int[,] heatMap = new int[640, 360];
    
    public List<Tuple<Dancer, Dancer>> SortDancersFromFrames(List<List<Frame>> posesByFrameByCamera)
    {
        List<Tuple<Dancer, Dancer>> dancersByCamera = new();

        int camCounter = 0;
        foreach (List<Frame> posesByFrame in posesByFrameByCamera)
        {
            Console.WriteLine("CAMERA " + camCounter);
            camCounter++;

            Dancer lead = new Dancer(Role.Lead);
            Dancer follow = new Dancer(Role.Follow);

            foreach (Frame frame in posesByFrame)
            {
                foreach (Rectangle rectangle in frame.BoundingBoxes)
                {
                    heatMap[rectangle.X, rectangle.Y]++;
                }
            }

            dancersByCamera.Add(new Tuple<Dancer, Dancer>(lead, follow));
        }

        return dancersByCamera;
    }

    static float Difference(IReadOnlyList<Point> a, IReadOnlyList<Point> b)
    {
        if (a.Count != b.Count) return float.MaxValue;
        return (float)a.Select((t, i) => Math.Sqrt(Math.Pow(t.X - b[i].X, 2) + Math.Pow(t.Y - b[i].Y, 2))).Sum();
    }
}