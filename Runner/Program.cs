using System.CommandLine;
using System.CommandLine.NamingConventionBinder;
using dancer_pose_alignment;

namespace Runner;

static class Program
{
    class Args
    {
        public string JsonInputDir { get; set; }
        public string JsonCameraSizes { get; set; }
        public string OutputDb { get; set; }
    }

    static void Main(string[] args)
    {
        RootCommand rootCommand =
        [
            new Argument<string>(
                "JsonInputDir",
                "Path to directory containing refined lead and follow poses"),


            new Argument<string>(
                "JsonCameraSizes",
                "Path to json file containing camera sizes"),


            new Argument<string>(
                "OutputDb",
                "Path to output sqlite database")
        ];

        rootCommand.Description =
            "Converts AlphaPose output to 3D poses, after refining each sequence and identifying lead and follow. ";

        rootCommand.Handler = CommandHandler.Create<Args>(Parse);
        rootCommand.Invoke(args);
        Environment.Exit(0);
    }

    static void Parse(Args args)
    {
        CameraPoseSolver.LoadPoses(args.JsonInputDir, args.JsonCameraSizes);
    }
}