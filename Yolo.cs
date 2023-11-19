using Compunet.YoloV8;
using Compunet.YoloV8.Data;
using SixLabors.ImageSharp;

namespace dancer_pose_alignment;

public static class Yolo
{
    public static List<List<Frame>> CalculatePosesFromImages(string inputPath)
    {
        List<List<Frame>> posesByFrameByCamera = new();

        ModelSelector modelSelector = new ModelSelector("yolov8x-pose.onnx");
        YoloV8 yolo = new(modelSelector);

        Point imgCenter = new Point(640 / 2, 360 / 2);

        int camCounter = 0;
        foreach (string directory in Directory.EnumerateDirectories(inputPath))
        {
            Console.WriteLine("CAMERA " + camCounter);
            camCounter++;
            
            // iterate through camera frames
            int frameCounter = 0;
            
            List<Frame> lastLeadPose = new();
            foreach (string filePath in Directory.EnumerateFiles(directory))
            {
                if(Path.GetExtension(filePath) != ".jpg") continue;
                Console.WriteLine("pose for frame " + frameCounter);

                ImageSelector imageSelector = new ImageSelector(filePath);
                IPoseResult result = yolo.Pose(imageSelector);

                Frame posesFromFrame = new(frameCounter);
                foreach (IPoseBoundingBox poseBoundingBox in result.Boxes)
                {
                    List<Point> pose = poseBoundingBox.Keypoints
                        .Select(kp => new Point(kp.Point.X - imgCenter.X, kp.Point.Y - imgCenter.Y))
                        .ToList();

                    posesFromFrame.Poses.Add(pose);
                    posesFromFrame.BoundingBoxes.Add(poseBoundingBox.Bounds);

                }
                
                lastLeadPose.Add(posesFromFrame);

                frameCounter++;
            }

            posesByFrameByCamera.Add(lastLeadPose);
        }

        return posesByFrameByCamera;
    }
}