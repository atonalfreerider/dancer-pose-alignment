using System.Numerics;

namespace dancer_pose_alignment;

public class CameraSetup
{
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

    public Vector3 Position;
    public Quaternion Rotation;
    public Vector2 Size;
    public float FocalLength = .2f;
    public Plane FocalPlane;

    public Vector3 Forward => Vector3.Transform(
        new Vector3(0, 0, 1),
        Rotation);

    public static void LoadCameraSetups(string positionPath)
    {
        Dictionary<string, PositionAndRotation> positionAndRotationByCam = Newtonsoft.Json.JsonConvert
            .DeserializeObject<Dictionary<string, PositionAndRotation>>(
                File.ReadAllText(positionPath));

        List<CameraSetup> cameras = positionAndRotationByCam.Values.Select(positionAndRotation =>
            new CameraSetup
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
    }

    static List<Vector3> Adjusted(IEnumerable<Vector3> keypoints, CameraSetup cam)
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