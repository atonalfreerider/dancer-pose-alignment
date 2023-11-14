using System.Numerics;
using Compunet.YoloV8;
using Compunet.YoloV8.Data;
using SixLabors.ImageSharp;

namespace dancer_pose_alignment;

public static class Yolo
{
    public static List<Tuple<Dancer, Dancer>> CalculatePosesFromImages(string inputPath)
    {
        List<Tuple<Dancer, Dancer>> dancersByCamera = new();

        ModelSelector modelSelector = new ModelSelector("yolov8x-pose.onnx");
        YoloV8 yolo = new(modelSelector);

        Point imgCenter = new Point(640 / 2, 360 / 2);

        int camCounter = 0;
        foreach (string directory in Directory.EnumerateDirectories(inputPath))
        {
            Console.WriteLine("CAMERA " + camCounter);
            camCounter++;

            Dancer lead = new Dancer(Role.Lead);
            Dancer follow = new Dancer(Role.Follow);

            // iterate through camera frames
            int frameCounter = 0;
            
            List<Point> lastLeadPose = new();
            List<Point> lastFollowPose = new();
            foreach (string filePath in Directory.EnumerateFiles(directory))
            {
                if(Path.GetExtension(filePath) != ".jpg") continue;
                Console.WriteLine("pose for frame " + frameCounter);

                ImageSelector imageSelector = new ImageSelector(filePath);
                IPoseResult result = yolo.Pose(imageSelector);

                int leadIdx = -1;
                int followIdx = -1;
                float largestArea = 0;
                float secondLargestArea = 0;
                List<List<Point>> posesFromFrame = new();
                foreach (IPoseBoundingBox poseBoundingBox in result.Boxes)
                {
                    List<Point> pose = poseBoundingBox.Keypoints
                        .Select(kp => new Point(kp.Point.X - imgCenter.X, kp.Point.Y - imgCenter.Y))
                        .ToList();

                    posesFromFrame.Add(pose);

                    float area = poseBoundingBox.Bounds.Width * poseBoundingBox.Bounds.Height;
                    if (area > largestArea)
                    {
                        secondLargestArea = largestArea;
                        largestArea = area;
                        followIdx = leadIdx;
                        leadIdx = posesFromFrame.Count - 1;
                    }
                    else if (area > secondLargestArea)
                    {
                        secondLargestArea = area;
                        followIdx = posesFromFrame.Count - 1;
                    }
                }

                if (frameCounter == 0)
                {
                    // pull out lead and follow
                    lastLeadPose = posesFromFrame[leadIdx];
                    lastFollowPose = posesFromFrame[followIdx];
                    lead.PosesByFrame.Add(lastLeadPose);
                    follow.PosesByFrame.Add(lastFollowPose);
                }
                else
                {
                    // compare to last frame
                    float lowestLeadDiff = float.MaxValue;
                    float lowestFollowDiff = float.MaxValue;
                    int lowestLeadIdx = -1;
                    int lowestFollowIdx = -1;
                    foreach (List<Point> pose in posesFromFrame)
                    {
                        float diffToLead = Difference(pose, lastLeadPose);
                        float diffToFollow = Difference(pose, lastFollowPose);

                        if (diffToLead < lowestLeadDiff)
                        {
                            lowestLeadDiff = diffToLead;
                            lowestLeadIdx = posesFromFrame.IndexOf(pose);
                        }
                        else if (diffToFollow < lowestFollowDiff)
                        {
                            lowestFollowDiff = diffToFollow;
                            lowestFollowIdx = posesFromFrame.IndexOf(pose);
                        }
                    }

                    // add to lead or follow
                    if (lowestLeadIdx != -1 && lowestLeadDiff < 1500)
                    {
                        Console.WriteLine($"Lead matched for frame {frameCounter}");
                        lastLeadPose = posesFromFrame[lowestLeadIdx];
                        lead.PosesByFrame.Add(lastLeadPose);
                    }
                    else
                    {
                        lead.PosesByFrame.Add(null);
                        Console.WriteLine(
                            $"Lead not matched for frame {frameCounter}: LowestLeadDiff: {lowestLeadDiff}");
                    }

                    if (lowestFollowIdx != -1 && lowestFollowDiff < 1500)
                    {
                        Console.WriteLine($"Follow matched for frame {frameCounter}");
                        lastFollowPose = posesFromFrame[lowestFollowIdx];
                        follow.PosesByFrame.Add(lastFollowPose);
                    }
                    else
                    {
                        follow.PosesByFrame.Add(null);
                        Console.WriteLine(
                            $"Follow not matched for frame {frameCounter}: LowestFollowDiff: {lowestFollowDiff}");
                    }
                }

                frameCounter++;
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

    static Point PtToPoint(Point pt)
    {
        return new Point(pt.X, pt.Y);
    }
}