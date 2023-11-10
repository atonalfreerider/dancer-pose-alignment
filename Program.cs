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

        
        SqliteOutput sqliteOutput = null;

        if (File.Exists(args.OutputDb))
        {
            // read the cached yolo 2d poses
            dancersByCamera = SqliteInput.ReadFrameFromDb(args.OutputDb);
            frameCount = SqliteInput.FRAME_MAX;
            Console.WriteLine("read from " + args.OutputDb + " with " + frameCount + " frames");
            
            // prepare to write the merged 3d poses
            sqliteOutput = new SqliteOutput(args.OutputDb, frameCount);
        }
        else
        {
            // calculate yolo 2d poses and cahce them
            dancersByCamera = Yolo.CalculatePosesFromImages(args.InputPath);
            frameCount = Yolo.frameCount;
            
            // cache the yolo 2d poses and prepare to write the merged 3d poses
            sqliteOutput = new SqliteOutput(args.OutputDb, frameCount);
            sqliteOutput.Serialize(dancersByCamera);
            Console.WriteLine("cached to " + args.OutputDb);
        }

        // merge into 3D poses
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

            foreach (Tuple<Dancer, Dancer> leadAndFollow in dancersByCamera)
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


        sqliteOutput.Serialize(new List<Tuple<Dancer, Dancer>> { new(lead, follow) });
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
}