using System.CommandLine;
using System.CommandLine.NamingConventionBinder;
using System.Numerics;
using Compunet.YoloV8;
using Compunet.YoloV8.Data;
using SixLabors.ImageSharp;

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
        Dancer lead = new Dancer()
        {
            Role = Role.Lead
        };
        Dancer follow = new Dancer()
        {
            Role = Role.Follow
        };
        
        List<Tuple<Dancer, Dancer>> dancersByCamera = new();
        foreach (string directory in Directory.EnumerateDirectories(args.InputPath))
        {
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
            foreach (string filePath in Directory.EnumerateFiles(directory))
            {
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
                
                List<Point> leadPoints = tallestBox.Keypoints.Select(kp => kp.Point).ToList();
                List<Vector3> leadPoints3d = leadPoints.Select(p => new Vector3(p.X, p.Y, 0)).ToList();
                leadForCam.PosesByFrame.Add(leadPoints3d);
                
                List<Point> followPoints = secondTallestBox.Keypoints.Select(kp => kp.Point).ToList();
                List<Vector3> followPoints3d = followPoints.Select(p => new Vector3(p.X, p.Y, 0)).ToList();
                followForCam.PosesByFrame.Add(followPoints3d);
                
                dancersByCamera.Add(new Tuple<Dancer, Dancer>(leadForCam, followForCam));
            }
        }
        
        // TODO merge 3d points from each camera for each dancer
        lead = dancersByCamera[0].Item1;
        follow = dancersByCamera[0].Item2;
        
        SqliteOutput sqliteOutput = new(args.OutputDb, frameCount);
        sqliteOutput.Serialize(new Tuple<Dancer, Dancer>(lead, follow));
    }
}