using System.Numerics;

namespace dancer_pose_alignment;

public class AlphaPose
{
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

    public static Dictionary<int, Dictionary<int, List<Vector3>>> PosesByFrameByPerson(string jsonPath)
    {
        List<SinglePose> singlePoses = Newtonsoft.Json.JsonConvert
            .DeserializeObject<List<SinglePose>>(File.ReadAllText(jsonPath));
            
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

        return posesByFrameByPerson;
    }
    
    public static void LoadAlphaPoseFromDirRoot(string dirRoot)
    {
        List<Tuple<Dancer, Dancer>> dancersByCamera = new();
        foreach (string dirPath in Directory.EnumerateDirectories(dirRoot))
        {
            string file = Path.Combine(dirPath, "alphapose-results.json");
            Dictionary<int, Dictionary<int, List<Vector3>>> posesByFrameByPerson = PosesByFrameByPerson(file);
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
    }
}