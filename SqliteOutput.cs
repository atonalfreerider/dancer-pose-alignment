using System.Data;
using System.Data.SQLite;
using System.Numerics;

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

        if (leadAndFollowPoses.Count == 1)
        {
            InsertDancer(conn, leadAndFollowPoses[0].Item1);
            InsertDancer(conn, leadAndFollowPoses[0].Item2);
        }
        else
        {
            Cache(conn, leadAndFollowPoses);
        }
    }

    static void CreateTables(SQLiteConnection conn)
    {
        using IDbCommand cmd = conn.CreateCommand();
        cmd.CommandText =
            """
            CREATE TABLE lead (
             id INTEGER PRIMARY KEY ASC,
             frame_id INTEGER NOT NULL,
             position_x REAL NOT NULL,
             position_y REAL NOT NULL,
             position_z REAL NOT NULL
            )
            """;
        cmd.ExecuteNonQuery();

        cmd.CommandText =
            """
            CREATE TABLE follow (
               id INTEGER PRIMARY KEY ASC,
               frame_id INTEGER NOT NULL,
               position_x REAL NOT NULL,
               position_y REAL NOT NULL,
               position_z REAL NOT NULL
             )
            """;
        cmd.ExecuteNonQuery();

        cmd.CommandText =
            """
            CREATE TABLE cache (
             id INTEGER PRIMARY KEY ASC,
             dancer_id INTEGER NOT NULL,
             camera_id INTEGER NOT NULL,
             frame_id INTEGER NOT NULL,
             position_x REAL NOT NULL,
             position_y REAL NOT NULL
            )
            """;
        cmd.ExecuteNonQuery();
    }

    static void Cache(
        IDbConnection conn,
        List<Tuple<Dancer, Dancer>> dancersByCamera)
    {
        using IDbCommand cmd = conn.CreateCommand();
        using IDbTransaction transaction = conn.BeginTransaction();
        cmd.CommandText =
            """
            INSERT INTO cache
             (
                 dancer_id,
                 camera_id,
                 frame_id,
                 position_x,
                 position_y
             ) VALUES (
                 @DancerId,
                 @CameraId,
                 @FrameId,
                 @PositionX,
                 @PositionY
             )
            """;

        int camCount = 0;
        foreach (Tuple<Dancer, Dancer> leadAndFollow in dancersByCamera)
        {
            int leadFrameCount = 0;
            foreach (List<Vector3> pose in leadAndFollow.Item1.PosesByFrame)
            {
                foreach (Vector3 position in pose)
                {
                    IDbDataParameter dancerIdParameter =
                        cmd.CreateParameter();
                    dancerIdParameter.DbType = DbType.Int32;
                    dancerIdParameter.ParameterName = "@DancerId";
                    dancerIdParameter.Value = 0;
                    cmd.Parameters.Add(dancerIdParameter);

                    IDbDataParameter cameraIdParameter =
                        cmd.CreateParameter();
                    cameraIdParameter.DbType = DbType.Int32;
                    cameraIdParameter.ParameterName = "@CameraId";
                    cameraIdParameter.Value = camCount;
                    cmd.Parameters.Add(cameraIdParameter);

                    IDbDataParameter frameIdParameter =
                        cmd.CreateParameter();
                    frameIdParameter.DbType = DbType.Int32;
                    frameIdParameter.ParameterName = "@FrameId";
                    frameIdParameter.Value = leadFrameCount;
                    cmd.Parameters.Add(frameIdParameter);

                    IDbDataParameter positionXParameter =
                        cmd.CreateParameter();
                    positionXParameter.DbType = DbType.Double;
                    positionXParameter.ParameterName = "@PositionX";
                    positionXParameter.Value = position.X;
                    cmd.Parameters.Add(positionXParameter);

                    IDbDataParameter positionYParameter =
                        cmd.CreateParameter();
                    positionYParameter.DbType = DbType.Double;
                    positionYParameter.ParameterName = "@PositionY";
                    positionYParameter.Value = position.Y;
                    cmd.Parameters.Add(positionYParameter);

                    IDbDataParameter positionZParameter =
                        cmd.CreateParameter();
                    positionZParameter.DbType = DbType.Double;
                    positionZParameter.ParameterName = "@PositionZ";
                    positionZParameter.Value = position.Z;
                    cmd.Parameters.Add(positionZParameter);

                    cmd.ExecuteNonQuery();
                    
                }
                leadFrameCount++;
            }

            int followFrameCount = 0;
            foreach (List<Vector3> pose in leadAndFollow.Item2.PosesByFrame)
            {
                foreach (Vector3 position in pose)
                {
                    IDbDataParameter dancerIdParameter =
                        cmd.CreateParameter();
                    dancerIdParameter.DbType = DbType.Int32;
                    dancerIdParameter.ParameterName = "@DancerId";
                    dancerIdParameter.Value = 1;
                    cmd.Parameters.Add(dancerIdParameter);

                    IDbDataParameter cameraIdParameter =
                        cmd.CreateParameter();
                    cameraIdParameter.DbType = DbType.Int32;
                    cameraIdParameter.ParameterName = "@CameraId";
                    cameraIdParameter.Value = camCount;
                    cmd.Parameters.Add(cameraIdParameter);

                    IDbDataParameter frameIdParameter =
                        cmd.CreateParameter();
                    frameIdParameter.DbType = DbType.Int32;
                    frameIdParameter.ParameterName = "@FrameId";
                    frameIdParameter.Value = followFrameCount;
                    cmd.Parameters.Add(frameIdParameter);

                    IDbDataParameter positionXParameter =
                        cmd.CreateParameter();
                    positionXParameter.DbType = DbType.Double;
                    positionXParameter.ParameterName = "@PositionX";
                    positionXParameter.Value = position.X;
                    cmd.Parameters.Add(positionXParameter);

                    IDbDataParameter positionYParameter =
                        cmd.CreateParameter();
                    positionYParameter.DbType = DbType.Double;
                    positionYParameter.ParameterName = "@PositionY";
                    positionYParameter.Value = position.Y;
                    cmd.Parameters.Add(positionYParameter);

                    IDbDataParameter positionZParameter =
                        cmd.CreateParameter();
                    positionZParameter.DbType = DbType.Double;
                    positionZParameter.ParameterName = "@PositionZ";
                    positionZParameter.Value = position.Z;
                    cmd.Parameters.Add(positionZParameter);

                    cmd.ExecuteNonQuery();
                }
                followFrameCount++;
            }

            camCount++;
        }

        transaction.Commit();
    }

    void InsertDancer(
        IDbConnection conn,
        Dancer dancer)
    {
        using IDbCommand cmd = conn.CreateCommand();
        using IDbTransaction transaction = conn.BeginTransaction();
        cmd.CommandText =
            @"INSERT INTO" + (dancer.Role == Role.Lead ? " lead " : " follow ") + """
                
                                      (
                                          frame_id,
                                          position_x,
                                          position_y,
                                          position_z
                                      ) VALUES (
                                          @FrameId,
                                          @PositionX,
                                          @PositionY,
                                          @PositionZ
                                      )
                """;

        for (int i = 0; i < TotalFrames; i++)
        {
            if (dancer.PosesByFrame.Count <= i) break;
            List<Vector3> pose = dancer.PosesByFrame[i];
            foreach (Vector3 position in pose)
            {
                IDbDataParameter frameIdParameter =
                    cmd.CreateParameter();
                frameIdParameter.DbType = DbType.Int32;
                frameIdParameter.ParameterName = "@FrameId";
                frameIdParameter.Value = i;
                cmd.Parameters.Add(frameIdParameter);

                IDbDataParameter positionXParameter =
                    cmd.CreateParameter();
                positionXParameter.DbType = DbType.Double;
                positionXParameter.ParameterName = "@PositionX";
                positionXParameter.Value = position.X;
                cmd.Parameters.Add(positionXParameter);

                IDbDataParameter positionYParameter =
                    cmd.CreateParameter();
                positionYParameter.DbType = DbType.Double;
                positionYParameter.ParameterName = "@PositionY";
                positionYParameter.Value = position.Y;
                cmd.Parameters.Add(positionYParameter);

                IDbDataParameter positionZParameter =
                    cmd.CreateParameter();
                positionZParameter.DbType = DbType.Double;
                positionZParameter.ParameterName = "@PositionZ";
                positionZParameter.Value = position.Z;
                cmd.Parameters.Add(positionZParameter);

                cmd.ExecuteNonQuery();
            }
        }

        transaction.Commit();
    }
}