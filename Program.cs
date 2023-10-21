using System.CommandLine;
using System.CommandLine.NamingConventionBinder;
using Compunet.YoloV8;
using Compunet.YoloV8.Data;

namespace dancer_pose_alignment;

static class Program
{
    class Args
    {
        public string InputPath { get; set; }
        public string OutputDb { get; set; }
    }

    static void Main(string[] args)
    {
        RootCommand rootCommand = new()
        {
            new Argument<string>("InputPath"),

            new Argument<string>("OutputDb")
        };

        rootCommand.Description = "Merge dance poses into a single database with animations between frames";

        // Note that the parameters of the handler method are matched according to the names of the options 
        rootCommand.Handler = CommandHandler.Create<Args>(Parse);

        rootCommand.Invoke(args);

        Environment.Exit(0);
    }

    static void Parse(Args args)
    {
        ModelSelector modelSelector = new ModelSelector("yolov8x-pose.onnx");
        YoloV8 yolo = new(modelSelector);


        int frameCount = 0;
        foreach (string directory in Directory.EnumerateDirectories(args.InputPath))
        {
            int fileCount = Directory.EnumerateFiles(directory).Count();
            if (fileCount > frameCount)
            {
                frameCount = fileCount;
            }

            // iterate through camera frames
            foreach (string filePath in Directory.EnumerateFiles(directory))
            {
                ImageSelector imageSelector = new ImageSelector(filePath);
                IPoseResult result = yolo.Pose(imageSelector);

                int tallest = 0;
                IPoseBoundingBox tallestBox = null;
                int secondTallest = 0;
                IPoseBoundingBox secondTallestBox = null;

                foreach (IPoseBoundingBox poseBoundingBox in result.Boxes.Where(x => x.Class.Name == "person"))
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
                
                
            }
        }
        
        //OpenCvSharp.Cv2.TriangulatePoints();
    }
}