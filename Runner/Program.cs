using System.CommandLine;
using System.CommandLine.NamingConventionBinder;
using dancer_pose_alignment;

namespace Runner;

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
            new Argument<string>(
                "InputPath",
                "Path to directory containing AlphaPose outputs and camera positions json in root"),

            new Argument<string>(
                "OutputDb",
                "Path to output sqlite database")
        };

        rootCommand.Description =
            "Converts AlphaPose output to 3D poses, after refining each sequence and identifying lead and follow. ";

        rootCommand.Handler = CommandHandler.Create<Args>(Parse);
        rootCommand.Invoke(args);
        Environment.Exit(0);
    }

    static void Parse(Args args)
    {
        CameraSetup.LoadCameraSetups(Path.Combine(args.InputPath, "positions.json"));
        AlphaPose.LoadAlphaPoseFromDirRoot(args.InputPath);
    }
}