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
            new Argument<string>("InputPath", "Path to directory containing images by camera"),

            new Argument<string>("OutputDb"),

            new Argument<string>("CameraPositions", "colmap format")
        };

        rootCommand.Description = "take yolov8 2d poses and project them into space to make 3D model. requires yolo8x-pose.onnx";

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
        public float rotationW;
    }

    class Camera
    {
        public Vector3 Position;
        public Quaternion Rotation;
        public Vector3 Forward => Vector3.Transform(
            new Vector3(0f, 0f, 1f),
            Rotation);
    }

    static void Parse(Args args)
    {
        Dictionary<string, PositionAndRotation> positionAndRotationByCam = Newtonsoft.Json.JsonConvert
            .DeserializeObject<Dictionary<string, PositionAndRotation>>(File.ReadAllText(args.CameraPositions))!;

        List<Camera> cameras = positionAndRotationByCam.Values.Select(positionAndRotation =>
            new Camera
            {
                Position = new Vector3(
                    positionAndRotation.positionX,
                    positionAndRotation.positionY,
                    positionAndRotation.positionZ),
                Rotation = new Quaternion(
                    positionAndRotation.rotationX,
                    positionAndRotation.rotationY,
                    positionAndRotation.rotationZ,
                    positionAndRotation.rotationW)
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

        int camCounter = 0;

        List<Tuple<Dancer, Dancer>> dancersByCamera = new();
        foreach (string directory in Directory.EnumerateDirectories(args.InputPath))
        {
            Console.WriteLine("CAMERA " + camCounter);
            camCounter++;

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
            int frameCounter = 0;
            foreach (string filePath in Directory.EnumerateFiles(directory))
            {
                frameCounter++;
                Console.WriteLine("pose for frame " + frameCounter);

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

                const float xCenter = 640f / 2;
                const float yCenter = 360f / 2;
                const float scale = 1.6f / 300f;

                List<Point> leadPoints = tallestBox.Keypoints.Select(kp => kp.Point).ToList();
                List<Vector3> leadPoints3d =
                    leadPoints.Select(p => new Vector3(p.X - xCenter, -p.Y + yCenter, 0) * scale).ToList();
                leadForCam.PosesByFrame.Add(leadPoints3d);

                List<Point> followPoints = secondTallestBox.Keypoints.Select(kp => kp.Point).ToList();
                List<Vector3> followPoints3d =
                    followPoints.Select(p => new Vector3(p.X - xCenter, -p.Y + yCenter, 0) * scale).ToList();
                followForCam.PosesByFrame.Add(followPoints3d);
            }

            dancersByCamera.Add(new Tuple<Dancer, Dancer>(leadForCam, followForCam));
        }

        // TODO iterate through each camera, and merge with the next camera
        for (int k = 0; k < camCounter - 1; k++)
        {
            Dancer cam0Lead = dancersByCamera[k].Item1;
            Dancer cam0Follow = dancersByCamera[k].Item2;

            Dancer cam1Lead = dancersByCamera[k + 1].Item1;
            Dancer cam1Follow = dancersByCamera[k + 1].Item2;

            for (int j = 0; j < frameCount; j++)
            {
                Console.WriteLine("triangulating frame " + j);
                List<Vector3> leadPose0 = cam0Lead.PosesByFrame[j];
                List<Vector3> followPose0 = cam0Follow.PosesByFrame[j];

                leadPose0 = Adjusted(leadPose0, cameras[k]);
                followPose0 = Adjusted(followPose0, cameras[k]);

                List<Vector3> leadPose1 = cam1Lead.PosesByFrame[j];
                List<Vector3> followPose1 = cam1Follow.PosesByFrame[j];

                leadPose1 = Adjusted(leadPose1, cameras[k + 1]);
                followPose1 = Adjusted(followPose1, cameras[k + 1]);

                List<Vector3> lead3DPosition = TriangulatedPose(
                    leadPose0,
                    leadPose1,
                    cameras[k].Forward,
                    cameras[k + 1].Forward);
                lead.PosesByFrame.Add(lead3DPosition);

                List<Vector3> follow3DPosition = TriangulatedPose(
                    followPose0,
                    followPose1,
                    cameras[k].Forward,
                    cameras[k + 1].Forward);
                follow.PosesByFrame.Add(follow3DPosition);
            }
        }

        SqliteOutput sqliteOutput = new(args.OutputDb, frameCount);
        sqliteOutput.Serialize(new Tuple<Dancer, Dancer>(lead, follow));

        Console.WriteLine("wrote to " + args.OutputDb);
    }

    static List<Vector3> Adjusted(IEnumerable<Vector3> keypoints, Camera cam)
    {
        // Translate keypoints to the camera center
        Vector3 cameraCenter = cam.Position;
        List<Vector3> adjustedKeypoints = keypoints.Select(t => cameraCenter + t).ToList();

        // Rotate keypoints around the camera center by the camera's rotation quaternion
        Quaternion rotation = cam.Rotation;
        for (int i = 0; i < adjustedKeypoints.Count; i++)
        {
            adjustedKeypoints[i] = Vector3.Transform(adjustedKeypoints[i] - cameraCenter, rotation) + cameraCenter;
        }

        return adjustedKeypoints;
    }

    static List<Vector3> TriangulatedPose(
        IReadOnlyList<Vector3> pose0,
        IReadOnlyList<Vector3> pose1,
        Vector3 cam0Forward,
        Vector3 cam1Forward)
    {
        // Create a list to store the resulting 3D pose
        List<Vector3> triangulatedPose = new List<Vector3>();

        // Iterate over each keypoint in pose0 and pose1
        for (int i = 0; i < pose0.Count; i++)
        {
            Vector3 point0 = pose0[i];
            Vector3 point1 = pose1[i];

            triangulatedPose.Add(MinimumMidpoint(point0, cam0Forward, point1, cam1Forward));
        }

        return triangulatedPose;
    }

    static Vector3 MinimumMidpoint(Vector3 P1, Vector3 D1, Vector3 P2, Vector3 D2)
    {
        Vector3 w0 = P1 - P2;
        float a = Vector3.Dot(D1, D1);
        float b = Vector3.Dot(D1, D2);
        float c = Vector3.Dot(D2, D2);
        float d = Vector3.Dot(D1, w0);
        float e = Vector3.Dot(D2, w0);

        float denom = a * c - b * b;

        float t = (b * e - c * d) / denom;
        float s = (a * e - b * d) / denom;

        Vector3 pointOnRay1 = P1 + t * D1;
        Vector3 pointOnRay2 = P2 + s * D2;

        return 0.5f * (pointOnRay1 + pointOnRay2);
    }
}