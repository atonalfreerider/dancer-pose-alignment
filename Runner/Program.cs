using System.Numerics;
using Compunet.YoloV8.Data;
using dancer_pose_alignment;
using Newtonsoft.Json;
using OpenCvSharp;

static class Program
{
    static void Main(string[] args)
    {
        string rootFolder = args[0];
        Yolo yolo = new("yolov8x-pose.onnx"); // this is in the assembly dir 
        Directory.CreateDirectory(rootFolder + "/pose");
        Directory.CreateDirectory(rootFolder + "/affine");

        const int sort_max_age = 5;
        const int sort_min_hits = 2;
        const float sort_iou_thresh = 0.2f;

        foreach (string videoPath in Directory.EnumerateFiles(rootFolder, "*.mp4"))
        {
            Console.WriteLine("affine " + videoPath);
            FrameSource frameSource = FrameSource.CreateFrameSource_Video(videoPath);
            List<List<List<Vector3>>> posesByFrame = [];
            List<Vector3> affineTransform = []; // translation x,y, rotation

            Affine affine = new();
            int frameCount = 0;
            int totalDetections = -1;
            
            KalmanFilterSort kalmanFilter = new(sort_max_age, sort_min_hits, sort_iou_thresh);
            while (true)
            {
                try
                {
                    OutputArray outputArray = new Mat();
                    Console.WriteLine(frameCount++);

                    frameSource.NextFrame(outputArray);

                    Mat frameMat = outputArray.GetMat();
                    List<IPoseBoundingBox> posesAndBoxesAtFrame = yolo
                        .CalculateBoxesAndPosesFromImage(frameMat.ToMemoryStream()).ToList();

                    float[][] tracked_dets = kalmanFilter.Update(posesAndBoxesAtFrame);
                    List<KalmanBoxTracker> tracks = kalmanFilter.GetTrackers();

                    List<List<Vector3>> posesAtFrameByTrack = [];
                    foreach (KalmanBoxTracker track in tracks)
                    {
                        if (track.Id > totalDetections)
                        {
                            totalDetections = track.Id;
                        }
                    }

                    for (int i = 0; i < totalDetections; i++)
                    {
                        bool matched = false;
                        foreach (KalmanBoxTracker track in tracks)
                        {
                            if (track.Id == i)
                            {
                                posesAtFrameByTrack.Add(track.LastKeypoints);
                                matched = true;
                                break;
                            }
                        }
                        
                        if(!matched)
                        {
                            posesAtFrameByTrack.Add([]);
                        }
                    }

                    posesByFrame.Add(posesAtFrameByTrack);

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

            string posePath = rootFolder + "/pose/" + Path.GetFileNameWithoutExtension(videoPath);
            string affinePath = rootFolder + "/affine/" + Path.GetFileNameWithoutExtension(videoPath);
            File.WriteAllText(posePath + ".mp4.json", JsonConvert.SerializeObject(posesByFrame));
            File.WriteAllText(affinePath + ".mp4.json", JsonConvert.SerializeObject(affineTransform));
        }
    }
}