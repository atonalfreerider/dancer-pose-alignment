using System.Numerics;
using Compunet.YoloV8;
using Compunet.YoloV8.Data;
using SixLabors.ImageSharp;

namespace dancer_pose_alignment;

public static class Yolo
{
    public static int frameCount = 0;
    
    public static List<Tuple<Dancer, Dancer>> CalculatePosesFromImages(string inputPath)
    {
        List<Tuple<Dancer, Dancer>> dancersByCamera = new();
        
        ModelSelector modelSelector = new ModelSelector("yolov8x-pose.onnx");
        YoloV8 yolo = new(modelSelector);

        int camCounter = 0;
        foreach (string directory in Directory.EnumerateDirectories(inputPath))
        {
            Console.WriteLine("CAMERA " + camCounter);
            camCounter++;

            int fileCount = Directory.EnumerateFiles(directory).Count();
            if (fileCount > frameCount)
            {
                frameCount = fileCount;
            }

            Dancer leadForCam = new Dancer()
            {
                Role = Role.Lead
            };
            Dancer followForCam = new Dancer()
            {
                Role = Role.Follow
            };

            // iterate through camera frames
            int frameCounter = 0;
            foreach (string filePath in Directory.EnumerateFiles(directory))
            {
                frameCounter++;
                Console.WriteLine("pose for frame " + frameCounter);

                ImageSelector imageSelector = new ImageSelector(filePath);
                IPoseResult result = yolo.Pose(imageSelector);

                int tallest = 0;
                IPoseBoundingBox tallestBox = null;
                int secondTallest = 0;
                IPoseBoundingBox secondTallestBox = null;

                foreach (IPoseBoundingBox poseBoundingBox in result.Boxes)
                {
                    int height = poseBoundingBox.Bounds.Height;
                    if (height > tallest)
                    {
                        secondTallest = tallest;
                        secondTallestBox = tallestBox;
                        tallest = height;
                        tallestBox = poseBoundingBox;
                    }
                    else if (height > secondTallest)
                    {
                        secondTallest = height;
                        secondTallestBox = poseBoundingBox;
                    }
                }

                const float xCenter = 640f / 2;
                const float yCenter = 360f / 2;
                const float scale = 1.6f / 300f;

                List<Point> leadPoints = tallestBox.Keypoints.Select(kp => kp.Point).ToList();
                List<Vector3> leadPoints3d =
                    leadPoints.Select(p => new Vector3(p.X - xCenter, -p.Y + yCenter, 0) * scale).ToList();
                leadForCam.PosesByFrame.Add(leadPoints3d);

                List<Point> followPoints = secondTallestBox.Keypoints.Select(kp => kp.Point).ToList();
                List<Vector3> followPoints3d =
                    followPoints.Select(p => new Vector3(p.X - xCenter, -p.Y + yCenter, 0) * scale).ToList();
                followForCam.PosesByFrame.Add(followPoints3d);
            }

            dancersByCamera.Add(new Tuple<Dancer, Dancer>(leadForCam, followForCam));
        }

        return dancersByCamera;
    }
}