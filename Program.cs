using System.CommandLine;
using System.CommandLine.NamingConventionBinder;
using System.Numerics;

namespace dancer_pose_alignment;

static class Program
{
    class Args
    {
        public string InputPath { get; set; }
        public string OutputDb { get; set; }
        public string CameraPositions { get; set; }
    }

    static void Main(string[] args)
    {
        RootCommand rootCommand = new()
        {
            new Argument<string>("InputPath", "Path to directory containing images by camera"),

            new Argument<string>("OutputDb"),

            new Argument<string>("CameraPositions", "Vector3 and Quaternion format")
        };

        rootCommand.Description =
            "take yolov8 2d poses and project them into space to make 3D model. requires yolo8x-pose.onnx";

        // Note that the parameters of the handler method are matched according to the names of the options 
        rootCommand.Handler = CommandHandler.Create<Args>(Parse);

        rootCommand.Invoke(args);

        Environment.Exit(0);
    }

    static void Parse(Args args)
    {
        // calculate yolo 2d poses and cahce them
        List<Tuple<Dancer, Dancer>> dancersByCamera = Yolo.CalculatePosesFromImages(args.InputPath);

        // cache the yolo 2d poses and prepare to write the merged 3d poses
        SqliteOutput sqliteOutput = new SqliteOutput(args.OutputDb, dancersByCamera.FirstOrDefault().Item1.PosesByFrame.Count);
        sqliteOutput.Serialize(dancersByCamera);
        Console.WriteLine("cached to " + args.OutputDb);
    }
}