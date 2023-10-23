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
        public float rotationW;
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

                List<Point> leadPoints = tallestBox.Keypoints.Select(kp => kp.Point).ToList();
                List<Vector3> leadPoints3d = leadPoints.Select(p => new Vector3(p.X, p.Y, 0)).ToList();
                leadForCam.PosesByFrame.Add(leadPoints3d);

                List<Point> followPoints = secondTallestBox.Keypoints.Select(kp => kp.Point).ToList();
                List<Vector3> followPoints3d = followPoints.Select(p => new Vector3(p.X, p.Y, 0)).ToList();
                followForCam.PosesByFrame.Add(followPoints3d);
            }
            
            dancersByCamera.Add(new Tuple<Dancer, Dancer>(leadForCam, followForCam));
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

        for (int j = 0; j < frameCount ; j++)
        {
            Console.WriteLine("triangulating frame " + j);
            List<Vector3> leadPose0 = cam0Lead.PosesByFrame[j];
            List<Vector3> followPose0 = cam0Follow.PosesByFrame[j];

            leadPose0 = Adjusted(leadPose0, cam0);
            followPose0 = Adjusted(followPose0, cam0);

            List<Vector3> leadPose1 = cam1Lead.PosesByFrame[j];
            List<Vector3> followPose1 = cam1Follow.PosesByFrame[j];

            leadPose1 = Adjusted(leadPose1, cam1);
            followPose1 = Adjusted(followPose1, cam1);

            List<Vector3> lead3DPosition = TriangulatedPose(leadPose0, leadPose1, forward0, forward1);
            lead.PosesByFrame.Add(lead3DPosition);

            List<Vector3> follow3DPosition = TriangulatedPose(followPose0, followPose1, forward0, forward1);
            follow.PosesByFrame.Add(follow3DPosition);
        }

        SqliteOutput sqliteOutput = new(args.OutputDb, frameCount);
        sqliteOutput.Serialize(new Tuple<Dancer, Dancer>(lead, follow));

        Console.WriteLine("wrote to " + args.OutputDb);
    }

    static List<Vector3> Adjusted(IReadOnlyCollection<Vector3> keypoints, Camera cam)
    {
        // Translate keypoints to the camera center
        Vector3 cameraCenter = cam.Position;
        List<Vector3> adjustedKeypoints = keypoints.Select(t => t - cameraCenter).ToList();

        // Rotate keypoints around the camera center by the camera's rotation quaternion
        Quaternion rotation = cam.Rotation;
        for (int i = 0; i < adjustedKeypoints.Count; i++)
        {
            adjustedKeypoints[i] = RotatePointAroundCameraCenter(adjustedKeypoints[i], cameraCenter, rotation);
        }

        return adjustedKeypoints;
    }

    static Vector3 RotatePointAroundCameraCenter(Vector3 point0, Vector3 point1, Quaternion q)
    {
        // Translate point0 to be relative to point1
        Vector3 relativePoint0 = point0 - point1;

        // Rotate the relative point using the quaternion
        Vector3 rotatedRelativePoint0 = Vector3.Transform(relativePoint0, q);

        // Translate the rotated relative point back to its original position
        Vector3 rotatedPoint0 = rotatedRelativePoint0 + point1;

        return rotatedPoint0;
    }

    static List<Vector3> TriangulatedPose(
        IReadOnlyList<Vector3> pose0, 
        IReadOnlyList<Vector3> pose1, 
        Vector3 cam0Forward,
        Vector3 cam1Forward)
    {
        // Define your threshold distance D
        const float thresholdDistance = 0.001f; // You can adjust this value

        // Create a list to store the resulting 3D pose
        List<Vector3> triangulatedPose = new List<Vector3>();

        // Iterate over each keypoint in pose0 and pose1
        for (int i = 0; i < pose0.Count; i++)
        {
            Console.WriteLine("triangulating keypoint " + i + " of " + pose0.Count);
            Vector3 point0 = pose0[i];
            Vector3 point1 = pose1[i];

            // Initialize step size, direction, and previous distance
            float stepSize = 1f; // Initial step size (you can adjust this)
            int direction = 1; // Initial direction
            float prevDistance = Vector3.Distance(point0, point1);

            int iterationCounter = 0;
            while (true)
            {
                iterationCounter++;
                
                // Calculate the 3D positions based on the current step
                point0 = CalculateProjectedPosition(cam0Forward, point0, stepSize * direction);
                point1 = CalculateProjectedPosition(cam1Forward, point1, stepSize * direction);

                // Calculate the distance between projected0 and projected1
                float distance = Vector3.Distance(point0, point1);
                
                if (iterationCounter > 1000 || distance < thresholdDistance)
                {
                    triangulatedPose.Add(Midpoint(point0, point1));
                    break;
                }

                if (distance > prevDistance)
                {
                    // flip and reduce
                    direction *= -1;
                    stepSize *= 0.5f;
                }

                // Update the previous distance
                prevDistance = distance;
            }
        }

        return triangulatedPose;
    }

    static Vector3 CalculateProjectedPosition(Vector3 forward, Vector3 point, float stepSize)
    {
        // Calculate the projected position based on forward vector and step size
        return point + (forward * stepSize);
    }
    
    static Vector3 Midpoint(Vector3 point0, Vector3 point1)
    {
        return (point0 + point1) / 2f;
    }
}