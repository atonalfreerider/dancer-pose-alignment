using System.CommandLine;
using System.CommandLine.NamingConventionBinder;
using System.Numerics;
using dancer_pose_alignment;
using Newtonsoft.Json;
using OpenCvSharp;

namespace Runner;

static class Program
{
    static void Main(string[] args)
    {
        RootCommand rootCommand = new("CLI for dancer pose alignment preparation. Run affine, or yolo")
        {
            new Argument<string>("rootFolder", "root folder containing videos and poses")
        };
        
        Command affineCommand = new("affine", "affine transform videos to align poses")
        {
            rootCommand.Arguments[0]
        };
        
        affineCommand.Handler = CommandHandler.Create<string>(Affine);
        
        rootCommand.AddCommand(affineCommand);
        
        Command yoloCommand = new("yolo", "convert yolo json to sqlite")
        {
            rootCommand.Arguments[0]
        };
        
        yoloCommand.Handler = CommandHandler.Create<string>(SaveYoloJsonToSqlite);
        
        rootCommand.AddCommand(yoloCommand);

        rootCommand.Invoke(args);
    }

    static void SaveYoloJsonToSqlite(string poseFolder)
    {
        List<string> poseJsons = Directory.EnumerateFiles(poseFolder, "*.json").ToList();
        List<string> fileNames = poseJsons.Select(Path.GetFileNameWithoutExtension).ToList();
        const string sqlitePath = @"C:\Users\john\Desktop\larissa-kadu-recap.db";
        if (File.Exists(sqlitePath))
        {
            File.Delete(sqlitePath);
        }
        SqliteOutput sqliteOutput = new(sqlitePath);
        sqliteOutput.CreateTables(fileNames);
        foreach (string posePath in poseJsons)
        {
            Console.WriteLine($"writing {posePath} to db");
            List<List<PoseBoundingBox>> posesByFrame = JsonConvert.DeserializeObject<List<List<PoseBoundingBox>>>(
                File.ReadAllText(posePath));

            foreach (List<PoseBoundingBox> poseBoundingBoxes in posesByFrame)
            {
                foreach (PoseBoundingBox poseBoundingBox in poseBoundingBoxes)
                {
                    foreach (Keypoint keypoint in poseBoundingBox.Keypoints)
                    {
                        if (keypoint.Point.X < 1 && keypoint.Point.Y < 1)
                        {
                            // set undetected keypoints to 0 confidence
                            // for some reason, these types are output by the python ultralytics library
                            // the yolonet library moves the keypoints to the closest joint
                            keypoint.Confidence = 0;
                        }
                    }
                }
            }

            sqliteOutput.Serialize(Path.GetFileNameWithoutExtension(posePath), posesByFrame);
        }
        
        Console.WriteLine($"wrote to {sqlitePath}");
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
            File.WriteAllText(affinePath + ".mp4.json", JsonConvert.SerializeObject(affineTransform, Formatting.Indented));
        }
    }
}