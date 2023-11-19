using System.Data;
using System.Data.SQLite;
using SixLabors.ImageSharp;

namespace dancer_pose_alignment;

public static class SqliteInput
{
    public static int FRAME_MAX = -1;

    public static List<List<Frame>> ReadPosesByFrameByCameraFromDb(string dbPath)
    {
        List<List<Frame>> posesByFrameByCamera = new();

        string connectionString = "URI=file:" + dbPath;

        int lastCameraId = 0;
        int lastFrameId = 0;
        List<Frame> currentCamera = new();
        Frame currentFrame = new(0);
        List<Point> currentPose = new();

        using IDbConnection conn = new SQLiteConnection(connectionString);
        conn.Open();

        List<string> columnNames = new List<string>
        {
            "id", "camera_id", "frame_id", "position_x", "position_y"
        };

        using IDbCommand cmd = conn.CreateCommand();
        cmd.CommandText = CommandString(columnNames, "cache_poses");

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

                currentCamera = new List<Frame>();
                posesByFrameByCamera.Add(currentCamera);
            }

            if (frameId != lastFrameId)
            {
                currentFrame = new Frame(frameId);
                currentCamera.Add(currentFrame);

                lastFrameId = frameId;
            }
            
            if (poseCount % 17 == 0)
            {
                currentPose = new List<Point>();
                currentFrame.Poses.Add(currentPose);
            }
            
            currentPose.Add(position);
            
            poseCount++;
        }

        return posesByFrameByCamera;
    }

    public static List<List<Frame>> ReadBoxesByFrameByCameraFromDb(
        string dbPath,
        List<List<Frame>> posesByFrameByCamera)
    {
        string connectionString = "URI=file:" + dbPath;

        int lastCameraId = 0;
        int lastFrameId = 0;
        List<Frame> currentCamera = posesByFrameByCamera[lastCameraId];
        Frame currentFrame = currentCamera[lastFrameId];

        using IDbConnection conn = new SQLiteConnection(connectionString);
        conn.Open();

        List<string> columnNames = new List<string>
        {
            "id", "camera_id", "frame_id", "position_x", "position_y", "width", "height"
        };

        using IDbCommand cmd = conn.CreateCommand();
        cmd.CommandText = CommandString(columnNames, "cache_boxes");

        using IDataReader reader = cmd.ExecuteReader();
        Dictionary<string, int> indexes = ColumnIndexes(reader, columnNames);
        
        while (reader.Read())
        {
            int frameId = reader.GetInt32(indexes["frame_id"]);
            int cameraId = reader.GetInt32(indexes["camera_id"]);

            int centerX = reader.GetInt32(indexes["position_x"]);
            int centerY = reader.GetInt32(indexes["position_y"]);
            int width = reader.GetInt32(indexes["width"]);
            int height = reader.GetInt32(indexes["height"]);

            if (cameraId > lastCameraId)
            {
                lastCameraId = cameraId;

                currentCamera = posesByFrameByCamera[cameraId];
            }

            if (frameId != lastFrameId)
            {
                currentFrame = currentCamera[frameId];
                lastFrameId = frameId;
            }
            
            currentFrame.BoundingBoxes.Add(new Rectangle(centerX, centerY, width, height));
        }

        return posesByFrameByCamera;
    }

    static string CommandString(IEnumerable<string> columnNames, string tableName)
    {
        string cmd = columnNames.Aggregate(
            "SELECT ",
            (current, columnName) => current + $"{columnName}, ");

        // remove last comma 
        cmd = cmd.Substring(0, cmd.Length - 2) + " ";
        cmd += $"FROM {tableName}";

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