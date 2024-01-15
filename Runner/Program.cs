using System.Numerics;

using dancer_pose_alignment;
using Newtonsoft.Json;
using OpenCvSharp;

static class Program
{
    static void Main(string[] args)
    {
        string rootFolder = args[0];
        Directory.CreateDirectory(rootFolder + "/pose");
        Directory.CreateDirectory(rootFolder + "/affine");

        const int sort_max_age = 5;
        const int sort_min_hits = 2;
        const float sort_iou_thresh = 0.2f;

        foreach (string videoPath in Directory.EnumerateFiles(rootFolder, "*.mp4"))
        {
            Console.WriteLine("affine " + videoPath);
            FrameSource frameSource = FrameSource.CreateFrameSource_Video(videoPath);
            List<Dictionary<int, List<Vector3>>> posesByFrame = [];
            List<Vector3> affineTransform = []; // translation x,y, rotation

            Affine affine = new();
            int frameCount = 0;
            
            KalmanFilterSort kalmanFilter = new(sort_max_age, sort_min_hits, sort_iou_thresh);
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