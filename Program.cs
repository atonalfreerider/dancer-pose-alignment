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

            new Argument<string>("CameraPositions", "Vector3 and Quaternion format")
        };

        rootCommand.Description =
            "take yolov8 2d poses and project them into space to make 3D model. requires yolo8x-pose.onnx";

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

        List<Vector3> cameraForwards = new();
        for (int i = 0; i < dancersByCamera.Count; i++)
        {
            cameraForwards.Add(cameras[i].Forward);
        }

        for (int j = 0; j < frameCount; j++)
        {
            int camCount = 0;
            List<List<Vector3>> lead2DPoses = new();
            List<List<Vector3>> follow2DPoses = new();
         
            foreach (Tuple<Dancer,Dancer> leadAndFollow in dancersByCamera)
            {
                List<Vector3> leadPose = Adjusted(leadAndFollow.Item1.PosesByFrame[j], cameras[camCount]);
                List<Vector3> followPose = Adjusted(leadAndFollow.Item2.PosesByFrame[j], cameras[camCount]);
                
                lead2DPoses.Add(leadPose);
                follow2DPoses.Add(followPose);
                camCount++;
            }
            
            
            lead.PosesByFrame.Add(RayMidpointFinder.Merged3DPose(lead2DPoses, cameraForwards));
            follow.PosesByFrame.Add(RayMidpointFinder.Merged3DPose(follow2DPoses, cameraForwards));
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

    static class RayMidpointFinder
    {
        struct Ray
        {
            public readonly Vector3 Origin;
            public readonly Vector3 Direction;

            public Ray(Vector3 origin, Vector3 direction)
            {
                Origin = origin;
                Direction = direction;
            }
        }
        
        const float Tolerance = 0.0001f;
        const int MaxIterations = 1000;

        public static List<Vector3> Merged3DPose(List<List<Vector3>> Poses2D, IReadOnlyList<Vector3> CameraForwards)
        {
            List<Vector3> pose = new();
            
            int poseCount = Poses2D[0].Count;
            for (int i = 0; i < poseCount; i++)
            {
                List<Ray> rays = new();
                int camCounter = 0;
                foreach (List<Vector3> pose2D in Poses2D)
                {
                    Vector3 origin = pose2D[i];
                    Vector3 direction = CameraForwards[camCounter];
                    rays.Add(new Ray(origin, direction));
                    camCounter++;
                }

                Vector3 midpoint = FindMinimumMidpoint(rays);
                pose.Add(midpoint);
            }


            return pose;
        }

        static Vector3 FindMinimumMidpoint(List<Ray> rays)
        {
            Vector3 startPoint = AverageOrigins(rays);
            return Optimize(startPoint, rays);
        }

        static Vector3 Optimize(Vector3 startPoint, List<Ray> rays)
        {
            Vector3 currentPoint = startPoint;
            const float stepSize = 1.0f;

            for (int i = 0; i < MaxIterations; i++)
            {
                Vector3 gradient = ComputeGradient(currentPoint, rays);
                Vector3 nextPoint = currentPoint - stepSize * gradient;

                // Check if the objective function value has converged
                if (Math.Abs(ObjectiveFunction(nextPoint, rays) - ObjectiveFunction(currentPoint, rays)) < Tolerance)
                    break;

                currentPoint = nextPoint;
            }

            return currentPoint;
        }

        static Vector3 ComputeGradient(Vector3 point, List<Ray> rays)
        {
            Vector3 gradient = new Vector3(0, 0, 0);

            foreach (Ray ray in rays)
            {
                Vector3 w = point - ray.Origin;
                float dotProduct = Vector3.Dot(w, ray.Direction);
                Vector3 projection = dotProduct * ray.Direction;
                Vector3 diff = w - projection;

                // Calculate the gradient of the squared distance
                Vector3 grad = 2 * (diff - Vector3.Dot(diff, ray.Direction) * ray.Direction);

                // Add the gradient for this ray to the total gradient
                gradient += grad;
            }

            return gradient;
        }

        static float ObjectiveFunction(Vector3 point, IEnumerable<Ray> rays)
        {
            return rays.Select(ray => DistanceFromPointToRay(point, ray))
                .Select(distance => distance * distance)
                .Sum();
        }

        static float DistanceFromPointToRay(Vector3 point, Ray ray)
        {
            // Calculate the shortest distance from 'point' to 'ray'
            Vector3 w0 = point - ray.Origin;
            float c1 = Vector3.Dot(w0, ray.Direction);
            float c2 = Vector3.Dot(ray.Direction, ray.Direction);
            float b = c1 / c2;

            Vector3 pb = ray.Origin + b * ray.Direction;
            return Vector3.Distance(point, pb);
        }

        static Vector3 AverageOrigins(IReadOnlyCollection<Ray> rays)
        {
            Vector3 sum = new Vector3(0, 0, 0);
            sum = rays.Aggregate(sum, (current, ray) => current + ray.Origin);

            return sum / rays.Count;
        }
    }
}