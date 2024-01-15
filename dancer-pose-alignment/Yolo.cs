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
    
    public List<IPoseBoundingBox> CalculateBoxesAndPosesFromImage(Stream imageStream) 
    { 
        ImageSelector imageSelector = new(imageStream); 
        IPoseResult result = yolo.Pose(imageSelector); 
         
        return result.Boxes.ToList();
    } 
}