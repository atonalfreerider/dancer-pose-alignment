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

    /// <summary>
    /// AlphaPose: one pose of several per frame. This pose should persist across frames
    /// </summary>
    [Serializable]
    public class SinglePose
    {
        public string image_id; // name of the image eg 0.jpg, 1.jpg, 2.jpg
        public int category_id; // unknown
        public List<float> keypoints; // X, Y, confidence, X, Y, confidence, ... x 136 since it is HAPLE
        public float score; // confidence of the pose
        public List<float> box; // x1, y1, x2, y2
        public int idx; // alphapose's attempt to track the same person across frames
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
            .DeserializeObject<Dictionary<string, PositionAndRotation>>(
                File.ReadAllText(Path.Combine(args.InputPath, "positions.json")));

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

        List<Tuple<Dancer, Dancer>> dancersByCamera = new();
        foreach (string dirPath in Directory.EnumerateDirectories(args.InputPath))
        {
            string file = Path.Combine(dirPath, "alphapose-results.json");
            List<SinglePose> singlePoses = Newtonsoft.Json.JsonConvert
                .DeserializeObject<List<SinglePose>>(File.ReadAllText(file));
            
            Dictionary<int, Dictionary<int, List<Vector3>>> posesByFrameByPerson = new();
            foreach (SinglePose singlePose in singlePoses)
            {
                int frameId = int.Parse(Path.GetFileNameWithoutExtension(singlePose.image_id));
                int personId = singlePose.idx;

                if (!posesByFrameByPerson.ContainsKey(personId))
                {
                    posesByFrameByPerson.Add(personId, new Dictionary<int, List<Vector3>>());
                }

                if (!posesByFrameByPerson[personId].ContainsKey(frameId))
                {
                    posesByFrameByPerson[personId].Add(frameId, new List<Vector3>());
                }

                for (int i = 0; i < singlePose.keypoints.Count; i += 3)
                {
                    posesByFrameByPerson[personId][frameId].Add(new Vector3(
                        singlePose.keypoints[i], // x
                        singlePose.keypoints[i + 1], // y
                        singlePose.keypoints[i + 2])); // confidence
                }
            }

            foreach ((int personId, Dictionary<int, List<Vector3>> posesByFrame) in posesByFrameByPerson)
            {
                // TODO sort and consolidate
            }


            // TODO
            // identify lead and follow as the largest figures moving the most


            // TODO
            // identify the lead as the taller figure


            Dancer leadForCamera = new Dancer()
            {
                Role = Role.Lead
            };
            leadForCamera.PosesByFrame.Add(new List<Vector3>()); // TODO

            Dancer followForCamera = new Dancer()
            {
                Role = Role.Follow
            };
            followForCamera.PosesByFrame.Add(new List<Vector3>()); // TODO

            dancersByCamera.Add(new Tuple<Dancer, Dancer>(leadForCamera, followForCamera));
        }

        int frameCount = 0;
        Dancer lead = new Dancer()
        {
            Role = Role.Lead
        };
        Dancer follow = new Dancer()
        {
            Role = Role.Follow
        };

        SqliteOutput sqliteOutput = new SqliteOutput(args.OutputDb, frameCount);

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