using System.Numerics;
using Compunet.YoloV8;
using Compunet.YoloV8.Data;

namespace dancer_pose_alignment;

public class Yolo
{
    readonly YoloV8 yolo;
    public Yolo(string modelPath)
    {
        ModelSelector modelSelector = new(modelPath);
        yolo = new YoloV8(modelSelector);
    }

    public List<List<Vector3>> CalculatePosesFromImage(Stream imageStream)
    {
        ImageSelector imageSelector = new(imageStream);
        IPoseResult result = yolo.Pose(imageSelector);

        return result.Boxes
            .Select(poseBoundingBox => poseBoundingBox.Keypoints
            .Select(kp => new Vector3(kp.Point.X, kp.Point.Y, kp.Confidence))
            .ToList()).ToList();
    }
}