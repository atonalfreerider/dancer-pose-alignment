using System.Numerics;
using Newtonsoft.Json;

namespace dancer_pose_alignment;

public static class CameraPoseSolver
{
    public static void LoadPoses(string inputPath, string jsonCameraSizes)
    {
        List<Vector2> imageSizes = JsonConvert.DeserializeObject<List<Vector2>>(File.ReadAllText(jsonCameraSizes));

        List<List<List<Vector3>>> leadByCamera = [];
        List<List<List<Vector3>>> followByCamera = [];
        foreach (string jsonPath in Directory.EnumerateFiles(inputPath, "*.json"))
        {
            List<List<Vector3>> finalIndexListLeadAndFollow =
                JsonConvert.DeserializeObject<List<List<Vector3>>>(File.ReadAllText(jsonPath));

            if (jsonPath.Contains("lead"))
            {
                leadByCamera.Add(finalIndexListLeadAndFollow);
            }
            else
            {
                followByCamera.Add(finalIndexListLeadAndFollow);
            }
        }

        int frameWithHighestConfidence = FrameWithHighestConfidence(leadByCamera, followByCamera);
        List<CameraSetup> cameras = [];
        for (int i = 0; i < followByCamera.Count; i++)
        {
            List<Vector3> leadAtCamera = leadByCamera[i][frameWithHighestConfidence];
            List<Vector3> followAtCamera = followByCamera[i][frameWithHighestConfidence];

            // position cameras in a circle
            CameraSetup camera = new()
            {
                Size = imageSizes[i]
            };
            camera.AddPosesAndRecenterAndScaleToCamera(leadAtCamera, followAtCamera);

            float angle = -(float)i / followByCamera.Count * 2 * MathF.PI; // ccw
            const float radius = 3f;
            const float startingHeight = 1.5f;
            camera.PositionsPerFrame.Add(new Vector3(
                MathF.Sin(angle) * radius,
                startingHeight,
                MathF.Cos(angle) * radius));

            camera.RotationsPerFrame.Add(Transform.LookAt(
                new Vector3(0, startingHeight, 0),
                Quaternion.Identity,
                camera.PositionsPerFrame[0]));

            cameras.Add(camera);
        }

        List<List<Vector3>> planeProjectedPosesLead = [];
        List<List<Vector3>> planeProjectedPosesFollow = [];
        List<Vector3> cameraForwards = [];
        foreach (CameraSetup cameraSetup in cameras)
        {
            cameraSetup.Project(0);
            planeProjectedPosesLead.Add(cameraSetup.LeadProjectionsPerFrame[0]);
            planeProjectedPosesFollow.Add(cameraSetup.FollowProjectionsPerFrame[0]);
            cameraForwards.Add(cameraSetup.Forward(0));
        }

        List<Vector3> merged3DPoseLead = RayMidpointFinder.Merged3DPose(planeProjectedPosesLead, cameraForwards);
        List<Vector3> merged3DPoseFollow = RayMidpointFinder.Merged3DPose(planeProjectedPosesFollow, cameraForwards);
        foreach (CameraSetup cameraSetup in cameras)
        {
            float error = cameraSetup.Error(merged3DPoseLead, merged3DPoseFollow);
        }

        string jsonCameras = JsonConvert.SerializeObject(cameras, Formatting.Indented);
        File.WriteAllText(Path.Combine(@"C:\Users\john\Desktop", "cameras.json"), jsonCameras);
        Console.WriteLine("wrote cameras.json");

        string jsonMerged3DPose = JsonConvert.SerializeObject(merged3DPoseLead, Formatting.Indented);
        File.WriteAllText(Path.Combine(@"C:\Users\john\Desktop", "merged3DPoseLead.json"), jsonMerged3DPose);
        Console.WriteLine("wrote merged3DPoseLead.json");
        
        string jsonMerged3DPoseFollow = JsonConvert.SerializeObject(merged3DPoseFollow, Formatting.Indented);
        File.WriteAllText(Path.Combine(@"C:\Users\john\Desktop", "merged3DPoseFollow.json"), jsonMerged3DPoseFollow);
        Console.WriteLine("wrote merged3DPoseFollow.json");

        var x = 1;
    }

    static int FrameWithHighestConfidence(
        IReadOnlyList<List<List<Vector3>>> leadByCamera,
        IReadOnlyList<List<List<Vector3>>> followByCamera)
    {
        int frameWithHighestConfidence = 0;
        float highestConfidence = 0f;
        for (int i = 0; i < leadByCamera.Count; i++)
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

    static float Confidence(IEnumerable<Vector3> lead, IEnumerable<Vector3> follow)
    {
        return lead.Sum(vector3 => vector3.Z) + follow.Sum(vector3 => vector3.Z);
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