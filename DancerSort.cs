using SixLabors.ImageSharp;

namespace dancer_pose_alignment;

public static class DancerSort
{
    public static List<Tuple<Dancer, Dancer>> CalculatePosesFromImages(
        List<List<List<List<Point>>>> posesByFrameByCamera)
    {
        List<Tuple<Dancer, Dancer>> dancersByCamera = new();

        int camCounter = 0;
        foreach (List<List<List<Point>>> posesByFrame in posesByFrameByCamera)
        {
            Console.WriteLine("CAMERA " + camCounter);
            camCounter++;

            Dancer lead = new Dancer(Role.Lead);
            Dancer follow = new Dancer(Role.Follow);

            // iterate through camera frames
            int frameCounter = 0;

            List<Point> lastLeadPose = new();
            List<Point> lastFollowPose = new();
            foreach (List<List<Point>> poses in posesByFrame)
            {
                if (frameCounter == 0)
                {
                    int leadIdx = -1;
                    int followIdx = -1;

                    float largestArea = 0;
                    float secondLargestArea = 0;

                    foreach (List<Point> pose in poses)
                    {
                        float area = Spread(pose);
                        if (area > largestArea)
                        {
                            secondLargestArea = largestArea;
                            largestArea = area;
                            followIdx = leadIdx;
                            leadIdx = poses.Count - 1;
                        }
                        else if (area > secondLargestArea)
                        {
                            secondLargestArea = area;
                            followIdx = poses.Count - 1;
                        }
                    }
                    
                    // pull out lead and follow
                    lastLeadPose = poses[leadIdx];
                    lastFollowPose = poses[followIdx];
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
                    foreach (List<Point> pose in poses)
                    {
                        float diffToLead = Difference(pose, lastLeadPose);
                        float diffToFollow = Difference(pose, lastFollowPose);

                        if (diffToLead < lowestLeadDiff)
                        {
                            lowestLeadDiff = diffToLead;
                            lowestLeadIdx = poses.IndexOf(pose);
                        }
                        else if (diffToFollow < lowestFollowDiff)
                        {
                            lowestFollowDiff = diffToFollow;
                            lowestFollowIdx = poses.IndexOf(pose);
                        }
                    }

                    // add to lead or follow
                    if (lowestLeadIdx != -1 && lowestLeadDiff < 1500)
                    {
                        Console.WriteLine($"Lead matched for frame {frameCounter}");
                        lastLeadPose = poses[lowestLeadIdx];
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
                        lastFollowPose = poses[lowestFollowIdx];
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

    static float Spread(IReadOnlyCollection<Point> points)
    {
        if (points.Count == 0)
            return 0;

        // Calculate the center (mean) of the points
        float centerX = (float)points.Average(pt => pt.X);
        float centerY = (float)points.Average(pt => pt.Y);

        // Calculate the sum of squared distances from the center
        float sumSquaredDistances = points.Sum(pt => 
            (pt.X - centerX) * (pt.X - centerX) + 
            (pt.Y - centerY) * (pt.Y - centerY));

        // Calculate the average squared distance and then take the square root
        return (float)Math.Sqrt(sumSquaredDistances / points.Count);
    }
}