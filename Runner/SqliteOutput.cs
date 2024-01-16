using System.Data;
using System.Data.Common;
using System.Data.SQLite;
using dancer_pose_alignment;
using Newtonsoft.Json;

namespace Runner;

public class SqliteOutput(string dbPath)
{
    public void CreateTables(int numTables)
    {
        using DbConnection conn = new SQLiteConnection($"URI=file:{dbPath}");
        conn.Open();
        using IDbCommand cmd = conn.CreateCommand();
        for (int i = 0; i < numTables; i++)
        {
            cmd.CommandText = $"CREATE TABLE table_{i} (" + @"
                          id INTEGER PRIMARY KEY ASC,
                          frame INTEGER NOT NULL,
                          keypoints TEXT NOT NULL,
                          bounds TEXT NOT NULL)";

            cmd.ExecuteNonQuery();
        }
    }

    public void Serialize(int tableNumber, List<List<PoseBoundingBox>> posesByFrame)
    {
        using DbConnection conn = new SQLiteConnection($"URI=file:{dbPath}");
        conn.Open();

        int frameNumber = 0;
        IDbCommand cmd = conn.CreateCommand();
        IDbTransaction transaction = conn.BeginTransaction();
        foreach (List<PoseBoundingBox> poseBoundingBoxes in posesByFrame)
        {
            foreach (PoseBoundingBox poseBoundingBox in poseBoundingBoxes)
            {
                cmd.CommandText =
                    $"INSERT INTO table_{tableNumber}(" + @" 
                          frame,
                          keypoints,
                          bounds
                      ) VALUES ( 
                          @Frame,
                          @Keypoints,
                          @Bounds)";


                cmd.AddParameter(DbType.Int32, "@Frame", frameNumber);
                cmd.AddParameter(DbType.String, "@Keypoints", JsonConvert.SerializeObject(poseBoundingBox.Keypoints));
                cmd.AddParameter(DbType.String, "@Bounds", JsonConvert.SerializeObject(poseBoundingBox.Bounds));
                cmd.ExecuteNonQuery();
            }

            frameNumber++;
        }

        transaction.Commit();
    }
}

static class SqliteExtension
{
    public static void AddParameter(this IDbCommand cmd, DbType type, string name, object value)
    {
        IDbDataParameter p = cmd.CreateParameter();
        p.DbType = type;
        p.ParameterName = name;
        p.Value = value ?? DBNull.Value;
        cmd.Parameters.Add(p);
    }
}