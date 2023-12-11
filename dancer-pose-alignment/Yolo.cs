using System.Numerics;
using Compunet.YoloV8;
using Compunet.YoloV8.Data;

namespace dancer_pose_alignment;

public class Yolo
{
    readonly YoloV8 yolo;
    public Yolo(string modelPath)
    {
        ModelSelector modelSelector = new ModelSelector(modelPath);
        yolo = new YoloV8(modelSelector);
    }

    public List<List<List<Vector3>>> CalculatePosesFromImages(string inputPath)
    {
        List<List<List<Vector3>>> posesByCam = [];

        foreach (string filePath in Directory.EnumerateFiles(inputPath, "*.jpg"))
        {
            ImageSelector imageSelector = new ImageSelector(filePath);
            IPoseResult result = yolo.Pose(imageSelector);

            List<List<Vector3>> poses = result.Boxes
                .Select(poseBoundingBox => poseBoundingBox.Keypoints
                .Select(kp => new Vector3(kp.Point.X, kp.Point.Y, kp.Confidence))
                .ToList()).ToList();

            posesByCam.Add(poses);
        }

        return posesByCam;
    }
}