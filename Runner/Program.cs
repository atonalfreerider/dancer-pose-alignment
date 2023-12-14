using System.Numerics;
using Compunet.YoloV8;
using Compunet.YoloV8.Data;
using Newtonsoft.Json;
using OpenCvSharp;

static class Program
{
    static void Main(string[] args)
    {
        Yolo yolo = new("yolov8x-pose.onnx"); // this is in the assembly dir 

        foreach (string videoPath in Directory.EnumerateFiles(args[0], "*.mp4"))
        {
            Console.WriteLine("posing " + videoPath);
            FrameSource frameSource = FrameSource.CreateFrameSource_Video(videoPath);
            List<List<List<Vector3>>> posesByFrame = [];
            
            int frameCount = 0;
            while (true)
            {
                try
                {
                    OutputArray outputArray = new Mat();
                    Console.WriteLine(frameCount++);
                    frameSource.NextFrame(outputArray);

                    Mat frameMat = outputArray.GetMat();
                    List<List<Vector3>> posesAtFrame = yolo.CalculatePosesFromImage(frameMat.ToMemoryStream()).ToList();
                    posesByFrame.Add(posesAtFrame);
                }
                catch (Exception e)
                {
                    Console.WriteLine(e);
                    break;
                }
            }
            
            File.WriteAllText(videoPath + ".json", JsonConvert.SerializeObject(posesByFrame));
        }
    }

    class Yolo
    {
        readonly YoloV8 yolo;

        public Yolo(string modelPath)
        {
            ModelSelector modelSelector = new ModelSelector(modelPath);
            yolo = new YoloV8(modelSelector);
        }

        public IEnumerable<List<Vector3>> CalculatePosesFromImage(Stream imageStream)
        {
            ImageSelector imageSelector = new ImageSelector(imageStream);
            IPoseResult result = yolo.Pose(imageSelector);

            return result.Boxes
                .Select(poseBoundingBox => poseBoundingBox.Keypoints
                    .Select(kp => new Vector3(kp.Point.X, kp.Point.Y, kp.Confidence))
                    .ToList()).ToList();
        }
    }
}