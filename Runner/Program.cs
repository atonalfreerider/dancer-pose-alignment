﻿using System.Numerics;
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

        foreach (string videoPath in Directory.EnumerateFiles(rootFolder, "*.mp4"))
        {
            Console.WriteLine("affine " + videoPath);
            FrameSource frameSource = FrameSource.CreateFrameSource_Video(videoPath);
            List<List<List<Vector3>>> posesByFrame = [];
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
                    List<List<Vector3>> posesAtFrame = yolo.CalculatePosesFromImage(frameMat.ToMemoryStream()).ToList();
                    posesByFrame.Add(posesAtFrame);

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