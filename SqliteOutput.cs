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
    }

    public void Serialize(Tuple<Dancer, Dancer> leadAndFollow)
    {
        if (string.IsNullOrEmpty(DbPath)) return;

        if (File.Exists(DbPath))
        {
            File.Delete(DbPath);
        }

        string cs = $"URI=file:{DbPath}";

        using SQLiteConnection conn = new(cs);
        conn.Open();

        using (IDbCommand cmd = conn.CreateCommand())
        {
            cmd.CommandText =
                @"CREATE TABLE lead (
                          id INTEGER PRIMARY KEY ASC,
                          frame_id INTEGER NOT NULL,
                          position_x REAL NOT NULL,
                          position_y REAL NOT NULL,
                          position_z REAL NOT NULL
                      )";
            cmd.ExecuteNonQuery();
            
            cmd.CommandText =
                @"CREATE TABLE follow (
                          id INTEGER PRIMARY KEY ASC,
                          frame_id INTEGER NOT NULL,
                          position_x REAL NOT NULL,
                          position_y REAL NOT NULL,
                          position_z REAL NOT NULL
                      )";
            cmd.ExecuteNonQuery();
        }

        InsertNodes(conn, leadAndFollow.Item1);
        InsertNodes(conn, leadAndFollow.Item2);
    }

    void InsertNodes(
        IDbConnection conn,
        Dancer dancer)
    {
        using IDbCommand cmd = conn.CreateCommand();
        using IDbTransaction transaction = conn.BeginTransaction();
        cmd.CommandText =
            @"INSERT INTO"+ (dancer.Role == Role.Lead ? " lead " : " follow ") + @"
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
                      )";

        for (int i = 0; i < TotalFrames; i++)
        {
            if(dancer.PosesByFrame.Count <= i) break;
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