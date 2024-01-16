using System.Numerics;
using dancer_pose_alignment;
using Newtonsoft.Json;
using OpenCvSharp;

namespace Runner;

static class Program
{
    static void Main(string[] args)
    {
        string rootFolder = args[0];
        SaveYoloJsonToSqlite(rootFolder);
    }

    static void SaveYoloJsonToSqlite(string poseFolder)
    {
        List<string> poseJsons = Directory.EnumerateFiles(poseFolder, "*.json").ToList();
        int numVideos = poseJsons.Count;
        const string sqlitePath = @"C:\Users\john\Desktop\larissa-kadu-recap.db";
        SqliteOutput sqliteOutput = new(sqlitePath);
        sqliteOutput.CreateTables(numVideos);
        int count = 0;
        foreach (string posePath in poseJsons)
        {
            Console.WriteLine($"writing {posePath} to db");
            List<List<PoseBoundingBox>> posesByFrame = JsonConvert.DeserializeObject<List<List<PoseBoundingBox>>>(
                File.ReadAllText(posePath));

            sqliteOutput.Serialize(count, posesByFrame);
            
            count++;
        }
        
        Console.WriteLine($"wrote {numVideos} to {sqlitePath}");
    }

    static void Affine(string rootFolder)
    {
        Directory.CreateDirectory(rootFolder + "/affine");

        foreach (string videoPath in Directory.EnumerateFiles(rootFolder, "*.mp4"))
        {
            Console.WriteLine("affine " + videoPath);
            FrameSource frameSource = FrameSource.CreateFrameSource_Video(videoPath);
            List<Vector3> affineTransform = []; // translation x,y, rotation

            Affine affine = new();
            int frameCount = 0;

            while (true)
            {
                try
                {
                    OutputArray outputArray = new Mat();
                    Console.WriteLine(frameCount++);

                    frameSource.NextFrame(outputArray);

                    Mat frameMat = outputArray.GetMat();
                    
                    if (frameCount == 1)
                    {
                        affine.Init(frameMat, videoPath);
                    }
                    else
                    {
                        Vector3 affineVector = affine.GetAffine(frameMat, videoPath);
                        affineTransform.Add(affineVector);
                    }
                }
                catch (Exception e)
                {
                    Console.WriteLine(e);
                    break;
                }
            }

            string affinePath = rootFolder + "/affine/" + Path.GetFileNameWithoutExtension(videoPath);
            File.WriteAllText(affinePath + ".mp4.json", JsonConvert.SerializeObject(affineTransform));
        }
    }
}