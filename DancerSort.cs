using SixLabors.ImageSharp;

namespace dancer_pose_alignment;

public class DancerSort
{
    public List<Tuple<Dancer, Dancer>> SortDancersFromFrames(List<List<Frame>> posesByFrameByCamera)
    {
        List<Tuple<Dancer, Dancer>> dancersByCamera = new();

        int camCounter = 0;
        foreach (List<Frame> posesByFrame in posesByFrameByCamera)
        {
            Console.WriteLine("CAMERA " + camCounter);
            camCounter++;

            List<Dancer> allFoundDancers = new();

            int frameCount = 0;
            foreach (Frame frame in posesByFrame)
            {
                int poseCount = 0;
                foreach (List<Point> pose in frame.Poses)
                {
                    Rectangle boundingBox = frame.BoundingBoxes[poseCount];
                    bool matched = false;
                    if (frameCount > 0)
                    {
                        foreach (Dancer dancer in allFoundDancers
                                     .OrderBy(x => x.NonOverlappingAreaWithLastBox(boundingBox, frameCount)))
                        {
                            if (boundingBox.Width * boundingBox.Height ==
                                dancer.NonOverlappingAreaWithLastBox(boundingBox, frameCount)) break; // box is not overlapping at all with top candidate
                            
                            matched = dancer.TryAddPose(frameCount, pose, boundingBox);
                            if (matched) break;
                        }
                    }

                    if (!matched)
                    {
                        Dancer newDancer = new Dancer(Role.Unknown);
                        if (frameCount > 0)
                        {
                            for (int i = 0; i < frameCount; i++)
                            {
                                // backfill missing poses
                                newDancer.TryAddPose(i, null, null);
                            }
                        }

                        matched = newDancer.TryAddPose(frameCount, pose, boundingBox);
                        if (!matched)
                        {
                            Console.WriteLine("ERROR: could not add pose to new dancer");
                        }

                        allFoundDancers.Add(newDancer);
                    }

                    poseCount++;
                }

                foreach (Dancer dancer in allFoundDancers)
                {
                    // top up
                    dancer.TryAddPose(frameCount, null, null);
                }

                frameCount++;
            }

            foreach (Dancer dancer in allFoundDancers)
            {
                dancer.InterpolateMissing();
            }

            // find the two dancers that are closes to the center of the image by taking dancer.AverageDistanceFromZeroForAllPoses()
            List<Dancer> top2 = allFoundDancers
                .OrderBy(dancer => dancer.AverageHeightForAllPoses())
                .Reverse()
                .Take(2)
                .ToList();
            Dancer lead = top2[0];
            lead.Role = Role.Lead;
            Dancer follow = top2[1];
            follow.Role = Role.Follow;
            dancersByCamera.Add(new Tuple<Dancer, Dancer>(lead, follow));
        }

        return dancersByCamera;
    }

    static float Difference(IReadOnlyCollection<Point> a, IReadOnlyList<Point> b)
    {
        if (a.Count != b.Count) return float.MaxValue;
        return (float)a.Select((t, i) => Math.Sqrt(Math.Pow(t.X - b[i].X, 2) + Math.Pow(t.Y - b[i].Y, 2))).Sum();
    }
}