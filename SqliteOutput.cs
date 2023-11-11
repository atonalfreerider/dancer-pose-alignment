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

    public SqliteOutput(string dbPath)
    {
        DbPath = dbPath;

        if (File.Exists(DbPath))
        {
            File.Delete(DbPath);
        }

        string cs = $"URI=file:{DbPath}";
        using SQLiteConnection conn = new(cs);
        conn.Open();
        CreateTables(conn);
    }


    static void CreateTables(SQLiteConnection conn)
    {
        using IDbCommand cmd = conn.CreateCommand();
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

    public void Serialize(List<List<List<List<Vector2>>>> allCameras)
    {
        string cs = $"URI=file:{DbPath}";
        using SQLiteConnection conn = new(cs);
        conn.Open();
        
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
        foreach (List<List<List<Vector2>>> camera in allCameras)
        {
            int frameCount = 0;
            foreach (List<List<Vector2>> frame in camera)
            {
                int dancerIdCount = 0;
                foreach (List<Vector2> pose in frame)
                {
                    foreach (Vector2 position in pose)
                    {
                        IDbDataParameter dancerIdParameter =
                            cmd.CreateParameter();
                        dancerIdParameter.DbType = DbType.Int32;
                        dancerIdParameter.ParameterName = "@DancerId";
                        dancerIdParameter.Value = dancerIdCount;
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
                        frameIdParameter.Value = frameCount;
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

                        cmd.ExecuteNonQuery();
                        
                     
                    }
                    dancerIdCount++;
                }

                frameCount++;
            }

            camCount++;
        }

        transaction.Commit();
    }
}