using System.Data;
using System.Data.SQLite;
using SixLabors.ImageSharp;

namespace dancer_pose_alignment;

public static class SqliteInput
{
    public static int FRAME_MAX = -1;

    public static List<List<List<List<Point>>>> ReadPosesByFrameByCameraFromDb(string dbPath)
    {
        List<List<List<List<Point>>>> posesByFrameByCamera = new();

        string connectionString = "URI=file:" + dbPath;

        int lastCameraId = 0;
        int lastLeadFrameId = 0;
        List<List<List<Point>>> currentCamera = new();
        List<List<Point>> currentFrame = new();
        List<Point> currentPose = new();

        using IDbConnection conn = new SQLiteConnection(connectionString);
        conn.Open();

        List<string> columnNames = new List<string>
        {
            "id", "camera_id", "frame_id", "position_x", "position_y"
        };

        using IDbCommand cmd = conn.CreateCommand();
        cmd.CommandText = CommandString(columnNames);

        using IDataReader reader = cmd.ExecuteReader();
        Dictionary<string, int> indexes = ColumnIndexes(reader, columnNames);
        int poseCount = 0;
        
        posesByFrameByCamera.Add(currentCamera);
        currentCamera.Add(currentFrame);
        while (reader.Read())
        {
            int frameId = reader.GetInt32(indexes["frame_id"]);
            if (frameId > FRAME_MAX)
            {
                FRAME_MAX = frameId;
            }

            int cameraId = reader.GetInt32(indexes["camera_id"]);

            Point position = new(
                reader.GetInt32(indexes["position_x"]),
                reader.GetInt32(indexes["position_y"]));

            if (cameraId > lastCameraId)
            {
                lastCameraId = cameraId;

                currentCamera = new List<List<List<Point>>>();
                posesByFrameByCamera.Add(currentCamera);
            }

            if (frameId != lastLeadFrameId)
            {
                currentFrame = new List<List<Point>>();
                currentCamera.Add(currentFrame);

                lastLeadFrameId = frameId;
            }
            
            if (poseCount % 17 == 0)
            {
                currentPose = new List<Point>();
                currentFrame.Add(currentPose);
            }
            
            currentPose.Add(position);
            
            poseCount++;
        }

        return posesByFrameByCamera;
    }

    static string CommandString(IEnumerable<string> columnNames)
    {
        string cmd = columnNames.Aggregate(
            "SELECT ",
            (current, columnName) => current + $"{columnName}, ");

        // remove last comma 
        cmd = cmd.Substring(0, cmd.Length - 2) + " ";
        cmd += $"FROM cache";

        return cmd;
    }

    static Dictionary<string, int> ColumnIndexes(IDataRecord reader, IEnumerable<string> columnNames)
    {
        return columnNames
            .ToDictionary(
                columnName => columnName,
                reader.GetOrdinal);
    }
}