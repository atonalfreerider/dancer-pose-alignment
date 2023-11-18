using System.CommandLine;
using System.CommandLine.NamingConventionBinder;
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
            new Argument<string>("InputPath", "Path to directory containing images by camera"),

            new Argument<string>("OutputDb")
        };

        rootCommand.Description = "take yolov8 2d poses and identify lead and follow. requires yolo8x-pose.onnx";

        // Note that the parameters of the handler method are matched according to the names of the options 
        rootCommand.Handler = CommandHandler.Create<Args>(Parse);

        rootCommand.Invoke(args);

        Environment.Exit(0);
    }

    static void Parse(Args args)
    {
        List<List<List<List<Point>>>> posesByFrameByCamera;
        SqliteOutput sqliteOutput = new SqliteOutput(args.OutputDb);
        if (sqliteOutput.TableExists("cache"))
        {
            // read the cached yolo 2d poses
            posesByFrameByCamera = SqliteInput.ReadPosesByFrameByCameraFromDb(args.OutputDb);
            Console.WriteLine("read from " + args.OutputDb + " with " + SqliteInput.FRAME_MAX + 1 + " frames");
        }
        else
        {
            // calculate yolo 2d poses and cahce them
            posesByFrameByCamera = Yolo.CalculatePosesFromImages(args.InputPath);
            sqliteOutput.Cache(posesByFrameByCamera);
            Console.WriteLine("cached to " + args.OutputDb);
        }

        List<Tuple<Dancer, Dancer>> dancersByCamera = DancerSort.CalculatePosesFromImages(posesByFrameByCamera);

        sqliteOutput.InsertDancers(dancersByCamera);
        Console.WriteLine("wrote dancers to " + args.OutputDb);
    }
}