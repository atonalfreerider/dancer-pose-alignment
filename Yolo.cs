using System.Numerics;
using Compunet.YoloV8;
using Compunet.YoloV8.Data;
using SixLabors.ImageSharp;

namespace dancer_pose_alignment;

public static class Yolo
{
    public static List<List<List<List<Vector2>>>> CalculatePosesFromImages(string inputPath)
    {
        List<List<List<List<Vector2>>>> dancersByCamera = new();
        
        ModelSelector modelSelector = new ModelSelector("yolov8x-pose.onnx");
        YoloV8 yolo = new(modelSelector);

        int camCounter = 0;
        foreach (string directory in Directory.EnumerateDirectories(inputPath))
        {
            Console.WriteLine("CAMERA " + camCounter);
            camCounter++;
 
            // iterate through camera frames
            int frameCounter = 0;
            List<List<List<Vector2>>> allFramesOfDancers = new();
            foreach (string filePath in Directory.EnumerateFiles(directory))
            {
                frameCounter++;
                Console.WriteLine("pose for frame " + frameCounter);

                ImageSelector imageSelector = new ImageSelector(filePath);
                IPoseResult result = yolo.Pose(imageSelector);

                List<List<Vector2>> frameOfDancers = new();
                foreach (IPoseBoundingBox poseBoundingBox in result.Boxes)
                {
                    const float xCenter = 640f / 2;
                    const float yCenter = 360f / 2;

                    List<Point> leadPoints = poseBoundingBox.Keypoints.Select(kp => kp.Point).ToList();
                    List<Vector2> leadPoints3d =
                        leadPoints.Select(p => new Vector2(p.X - xCenter, -p.Y + yCenter)).ToList();
                    frameOfDancers.Add(leadPoints3d);
                }
                
                allFramesOfDancers.Add(frameOfDancers);
            }

            dancersByCamera.Add(allFramesOfDancers);
        }

        return dancersByCamera;
    }
}