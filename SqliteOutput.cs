using System.Data;
using System.Data.SQLite;
using SixLabors.ImageSharp;

namespace dancer_pose_alignment;

/// <summary>
/// Writes the result to an external database.
/// </summary>
public class SqliteOutput
{
    readonly string DbPath;
    readonly int TotalFrames;

    public SqliteOutput(string dbPath, int totalFrames)
    {
        DbPath = dbPath;
        TotalFrames = totalFrames;

        if (File.Exists(DbPath)) return;

        string cs = $"URI=file:{DbPath}";
        using SQLiteConnection conn = new(cs);
        conn.Open();
        CreateTables(conn);
    }

    public void Serialize(List<Tuple<Dancer, Dancer>> leadAndFollowPoses)
    {
        string cs = $"URI=file:{DbPath}";
        using SQLiteConnection conn = new(cs);
        conn.Open();

        int cameraCount = 0;
        foreach (Tuple<Dancer,Dancer> leadAndFollowPose in leadAndFollowPoses)
        {
            InsertDancer(conn, leadAndFollowPose.Item1, cameraCount);
            InsertDancer(conn, leadAndFollowPose.Item2, cameraCount);
            cameraCount++;
        }
    }

    static void CreateTables(SQLiteConnection conn)
    {
        using IDbCommand cmd = conn.CreateCommand();
        cmd.CommandText =
            """
            CREATE TABLE lead (
             id INTEGER PRIMARY KEY ASC,
             camera_id INTEGER NOT NULL,
             frame_id INTEGER NOT NULL,
             position_x INTEGER,
             position_y INTEGER
            )
            """;
        cmd.ExecuteNonQuery();

        cmd.CommandText =
            """
            CREATE TABLE follow (
               id INTEGER PRIMARY KEY ASC,
               camera_id INTEGER NOT NULL,
               frame_id INTEGER NOT NULL,
               position_x INTEGER,
               position_y INTEGER
             )
            """;
        cmd.ExecuteNonQuery();
    }


    void InsertDancer(
        IDbConnection conn,
        Dancer dancer,
        int cameraId)
    {
        using IDbCommand cmd = conn.CreateCommand();
        using IDbTransaction transaction = conn.BeginTransaction();
        cmd.CommandText =
            @"INSERT INTO" + (dancer.Role == Role.Lead ? " lead " : " follow ") + 
            """
            (
                camera_id,
                frame_id,
                position_x,
                position_y
            ) VALUES (
                @CameraId,
                @FrameId,
                @PositionX,
                @PositionY
            )
            """;

        for (int i = 0; i < TotalFrames; i++)
        {
            if (dancer.PosesByFrame.Count <= i) break;
            List<Point>? pose = dancer.PosesByFrame[i];

            if (pose == null)
            {
                for (int j = 0; j < 17; j++)
                {
                    IDbDataParameter cameraIdParameter =
                        cmd.CreateParameter();
                    cameraIdParameter.DbType = DbType.Int32;
                    cameraIdParameter.ParameterName = "@CameraId";
                    cameraIdParameter.Value = cameraId;
                    cmd.Parameters.Add(cameraIdParameter);
                
                    IDbDataParameter frameIdParameter =
                        cmd.CreateParameter();
                    frameIdParameter.DbType = DbType.Int32;
                    frameIdParameter.ParameterName = "@FrameId";
                    frameIdParameter.Value = i;
                    cmd.Parameters.Add(frameIdParameter);

                    IDbDataParameter positionXParameter =
                        cmd.CreateParameter();
                    positionXParameter.DbType = DbType.Int32;
                    positionXParameter.ParameterName = "@PositionX";
                    positionXParameter.Value = null;
                    cmd.Parameters.Add(positionXParameter);

                    IDbDataParameter positionYParameter =
                        cmd.CreateParameter();
                    positionYParameter.DbType = DbType.Int32;
                    positionYParameter.ParameterName = "@PositionY";
                    positionYParameter.Value = null;
                    cmd.Parameters.Add(positionYParameter);

                    cmd.ExecuteNonQuery();
                }
            }
            else
            {
                foreach (Point position in pose)
                {
                    IDbDataParameter cameraIdParameter =
                        cmd.CreateParameter();
                    cameraIdParameter.DbType = DbType.Int32;
                    cameraIdParameter.ParameterName = "@CameraId";
                    cameraIdParameter.Value = cameraId;
                    cmd.Parameters.Add(cameraIdParameter);

                    IDbDataParameter frameIdParameter =
                        cmd.CreateParameter();
                    frameIdParameter.DbType = DbType.Int32;
                    frameIdParameter.ParameterName = "@FrameId";
                    frameIdParameter.Value = i;
                    cmd.Parameters.Add(frameIdParameter);

                    IDbDataParameter positionXParameter =
                        cmd.CreateParameter();
                    positionXParameter.DbType = DbType.Int32;
                    positionXParameter.ParameterName = "@PositionX";
                    positionXParameter.Value = position.X;
                    cmd.Parameters.Add(positionXParameter);

                    IDbDataParameter positionYParameter =
                        cmd.CreateParameter();
                    positionYParameter.DbType = DbType.Int32;
                    positionYParameter.ParameterName = "@PositionY";
                    positionYParameter.Value = position.Y;
                    cmd.Parameters.Add(positionYParameter);

                    cmd.ExecuteNonQuery();
                }
            }
        }

        transaction.Commit();
    }
}