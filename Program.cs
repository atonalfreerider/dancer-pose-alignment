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
        public string CameraPositions { get; set; }
    }

    static void Main(string[] args)
    {
        RootCommand rootCommand = new()
        {
            new Argument<string>("InputPath"),

            new Argument<string>("OutputDb"),

            new Argument<string>("CameraPositions")
        };

        rootCommand.Description = "Merge dance poses into a single database with animations between frames";

        // Note that the parameters of the handler method are matched according to the names of the options 
        rootCommand.Handler = CommandHandler.Create<Args>(Parse);

        rootCommand.Invoke(args);

        Environment.Exit(0);
    }

    [Serializable]
    public class PositionAndRotation
    {
        public float positionX;
        public float positionY;
        public float positionZ;

        public float rotationX;
        public float rotationY;
        public float rotationZ;
    }

    class Camera
    {
        public Vector3 Position;
        public Quaternion Rotation;
    }

    static void Parse(Args args)
    {
        Dictionary<string, PositionAndRotation> positionAndRotationByCam = Newtonsoft.Json.JsonConvert
            .DeserializeObject<Dictionary<string, PositionAndRotation>>(File.ReadAllText(args.CameraPositions));

        List<Camera> cameras = positionAndRotationByCam.Values.Select(positionAndRotation =>
            new Camera
            {
                Position = new Vector3(
                    positionAndRotation.positionX,
                    positionAndRotation.positionY,
                    positionAndRotation.positionZ),
                Rotation = Quaternion.CreateFromYawPitchRoll(
                    positionAndRotation.rotationX,
                    positionAndRotation.rotationY,
                    positionAndRotation.rotationZ)
            }).ToList();

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
            int count = 0;
            foreach (string filePath in Directory.EnumerateFiles(directory))
            {
                count++;
                if (count != 80) continue;
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
        
        Camera cam0 = cameras[0];
        Vector3 forward0 = Vector3.Transform(
            new Vector3(0f, 0f, 1f), // vector forward
            cam0.Rotation); // quaternion

        Dancer cam0Lead = dancersByCamera[0].Item1;
        Dancer cam0Follow = dancersByCamera[0].Item2;
        
        Camera cam1 = cameras[1];
        Vector3 forward1 = Vector3.Transform(
            new Vector3(0f, 0f, 1f), // vector forward
            cam1.Rotation); // quaternion

        Dancer cam1Lead = dancersByCamera[1].Item1;
        Dancer cam1Follow = dancersByCamera[1].Item2;

        for (int j = 0; j < frameCount; j++)
        {
            List<Vector3> leadPose0 = cam0Lead.PosesByFrame[j];
            List<Vector3> followPose0 = cam0Follow.PosesByFrame[j];
            
            List<Vector3> leadPose1 = cam1Lead.PosesByFrame[j];
            List<Vector3> followPose1 = cam1Follow.PosesByFrame[j];

            List<Vector3> lead3DPosition = TriangulatedPose(leadPose0, leadPose1, forward0, forward1);
            lead.PosesByFrame.Add(lead3DPosition);
            
            List<Vector3> follow3DPosition = TriangulatedPose(followPose0, followPose1, forward0, forward1);
            follow.PosesByFrame.Add(follow3DPosition);
        }

        SqliteOutput sqliteOutput = new(args.OutputDb, frameCount);
        sqliteOutput.Serialize(new Tuple<Dancer, Dancer>(lead, follow));

        Console.WriteLine("wrote to " + args.OutputDb);
    }

    static List<Vector3> TriangulatedPose(List<Vector3> pose0, List<Vector3> pose1, Vector3 cam0Forward, Vector3 cam1Forward)
    {
        return new List<Vector3>();
    }
}