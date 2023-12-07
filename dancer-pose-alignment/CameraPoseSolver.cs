using System.Numerics;
using Newtonsoft.Json;

namespace dancer_pose_alignment;

public static class CameraPoseSolver
{
    public static void LoadPoses(string inputPath)
    {
        List<List<List<Vector3>>> followByCamera = new();
        List<List<List<Vector3>>> leadByCamera = new();
        foreach (string jsonPath in Directory.EnumerateFiles(inputPath, "*.json"))
        {
            List<List<Vector3>> finalIndexListLeadAndFollow =
                JsonConvert.DeserializeObject<List<List<Vector3>>>(File.ReadAllText(jsonPath));

            if (jsonPath.Contains("follow"))
            {
                followByCamera.Add(finalIndexListLeadAndFollow);
            }
            else
            {
                leadByCamera.Add(finalIndexListLeadAndFollow);
            }
        }

        int frameWithHighestConfidence = FrameWithHighestConfidence(followByCamera, leadByCamera);
        List<List<Vector3>> follows = new();
        List<List<Vector3>> leads = new();
        List<CameraSetup> cameras = new();
        for (int i = 0; i < followByCamera.Count; i++)
        {
            List<Vector3> followAtCamera = followByCamera[i][frameWithHighestConfidence];
            List<Vector3> leadAtCamera = leadByCamera[i][frameWithHighestConfidence];

            follows.Add(followAtCamera);
            leads.Add(leadAtCamera);
            
            // position cameras in a circle
            CameraSetup camera = new();
            float angle = -(float)i / followByCamera.Count * 2 * MathF.PI; // ccw
            const float radius = 3f;
            const float startingHeight = 1.5f;
            camera.Position = new Vector3(
                MathF.Sin(angle) * radius,
                startingHeight,
                MathF.Cos(angle) * radius);

            camera.Rotation = Transform.LookAt(
                new Vector3(0, startingHeight, 0),
                Quaternion.Identity,
                camera.Position);
            cameras.Add(camera);
        }

        string jsonCameras = JsonConvert.SerializeObject(cameras, Formatting.Indented);
        File.WriteAllText(Path.Combine(@"C:\Users\john\Desktop", "cameras.json"), jsonCameras);

        var x = 1;
    }

    static int FrameWithHighestConfidence(
        IReadOnlyList<List<List<Vector3>>> followByCamera,
        IReadOnlyList<List<List<Vector3>>> leadByCamera)
    {
        int frameWithHighestConfidence = 0;
        float highestConfidence = 0f;
        for (int i = 0; i < followByCamera.Count; i++)
        {
            for (int j = 0; j < 896; j++)
            {
                float confidence = Confidence(followByCamera[i][j], leadByCamera[i][j]);
                if (confidence > highestConfidence)
                {
                    highestConfidence = confidence;
                    frameWithHighestConfidence = j;
                }
            }
        }

        return frameWithHighestConfidence;
    }

    static float Confidence(IEnumerable<Vector3> follow, IEnumerable<Vector3> lead)
    {
        return follow.Sum(vector3 => vector3.Z) + lead.Sum(vector3 => vector3.Z);
    }

    static Tuple<List<Vector3>, List<Vector3>> LeadAndFollow3DSolution(
        List<List<Vector3>> leads,
        List<List<Vector3>> follows,
        List<CameraSetup> cameras)
    {
        Tuple<List<Vector3>, List<Vector3>> leadAndFollow3DSolution = null;


        return leadAndFollow3DSolution;
    }
}