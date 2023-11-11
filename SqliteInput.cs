using System.Data;
using System.Data.SQLite;
using System.Numerics;

namespace dancer_pose_alignment;

public static class SqliteInput
{
    public static int FRAME_MAX = -1;
    
    public static List<Tuple<Dancer, Dancer>> ReadFrameFromDb(string dbPath)
    {
        List<Tuple<Dancer, Dancer>> dancersByCamera = new();
        
        string connectionString = "URI=file:" + dbPath;

        Dancer currentLead = new Dancer();
        Dancer currentFollower = new Dancer();

        int lastCameraId = -1;
        int lastLeadFrameId = -1;
        int lastFollowFrameId = -1;
        List<Vector3> currentLeadPoses = new();
        List<Vector3> currentFollowPoses = new();

        using IDbConnection conn = new SQLiteConnection(connectionString);
        conn.Open();

        List<string> columnNames = new List<string>
        {
            "id", "dancer_id", "camera_id", "frame_id", "position_x", "position_y"
        };

        using IDbCommand cmd = conn.CreateCommand();
        cmd.CommandText = CommandString(columnNames);

        using IDataReader reader = cmd.ExecuteReader();
        Dictionary<string, int> indexes = ColumnIndexes(reader, columnNames);
        while (reader.Read())
        {
            int frameId = reader.GetInt32(indexes["frame_id"]);
            if (frameId > FRAME_MAX)
            {
                FRAME_MAX = frameId;
            }

            int dancerId = reader.GetInt32(indexes["dancer_id"]);
            int cameraId = reader.GetInt32(indexes["camera_id"]);

            Vector3 position = new(
                reader.GetFloat(indexes["position_x"]),
                reader.GetFloat(indexes["position_y"]),
                0f);
                        
            if (cameraId > lastCameraId)
            {
                lastCameraId = cameraId;
                currentLead = new Dancer()
                {
                    Role = Role.Lead
                };
                currentFollower = new Dancer()
                {
                    Role = Role.Follow
                };
                            
                dancersByCamera.Add(new Tuple<Dancer, Dancer>(currentLead, currentFollower));
                
                currentLeadPoses = new List<Vector3>();
                currentFollowPoses = new List<Vector3>();
            }

            if (dancerId == 0)
            {
                if (frameId != lastLeadFrameId)
                {
                    if (currentLeadPoses.Any())
                    {
                        currentLead.PosesByFrame.Add(currentLeadPoses);
                    }

                    currentLeadPoses = new List<Vector3>();

                    lastLeadFrameId = frameId;
                }

                currentLeadPoses.Add(position);
            }
            else if (dancerId == 1)
            {
                if (frameId != lastFollowFrameId)
                {
                    if (currentFollowPoses.Any())
                    {
                        currentFollower.PosesByFrame.Add(currentFollowPoses);
                    }

                    currentFollowPoses = new List<Vector3>();

                    lastFollowFrameId = frameId;
                }

                currentFollowPoses.Add(position);
            }
        }

        return dancersByCamera;
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