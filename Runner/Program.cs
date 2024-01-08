using System.Numerics;
using Compunet.YoloV8;
using Compunet.YoloV8.Data;
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

            Dictionary<string, InputArray> previousImages = [];
            Dictionary<string, InputArray> previousTrackingPoints = [];

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

                    // optical flow for camera motion
                    InputArray toGray = InputArray.Create(frameMat);
                    OutputArray grayImg = new Mat();
                    Cv2.CvtColor(toGray, grayImg, ColorConversionCodes.BGR2GRAY);
                    InputArray toTrack = InputArray.Create(grayImg.GetMat());
                    Point2f[] points = Cv2.GoodFeaturesToTrack(
                        toTrack,
                        100,
                        0.01,
                        10,
                        null,
                        3,
                        false,
                        0.04);

                    Mat pointsMat = new Mat(points.Length, 1, MatType.CV_32FC2);
                    pointsMat.SetArray(points);

                    InputOutputArray pointsToTrack = InputOutputArray.Create(pointsMat);

                    if (frameCount == 1)
                    {
                        previousImages[videoPath] = toTrack;
                        previousTrackingPoints[videoPath] = pointsToTrack.GetMat();
                        continue;
                    }

                    OutputArray status = new Mat();
                    OutputArray err = new Mat();
                    Cv2.CalcOpticalFlowPyrLK(
                        previousImages[videoPath], // previous img
                        toTrack, // current img
                        previousTrackingPoints[videoPath],
                        pointsToTrack,
                        status,
                        err);

                    Mat transform =
                        Cv2.EstimateAffinePartial2D(previousTrackingPoints[videoPath], pointsToTrack.GetMat());

                    // Extract translation
                    double dx = transform.At<double>(0, 2);
                    double dy = transform.At<double>(1, 2);

                    // Extract rotation angle
                    double da = Math.Atan2(transform.At<double>(1, 0), transform.At<double>(0, 0));

                    Vector3 affine = new((float)dx, (float)dy, (float)da);
                    affineTransform.Add(affine);

                    previousImages[videoPath] = toTrack;
                    previousTrackingPoints[videoPath] = pointsToTrack.GetMat();
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

    class Yolo
    {
        readonly YoloV8 yolo;

        public Yolo(string modelPath)
        {
            ModelSelector modelSelector = new ModelSelector(modelPath);
            yolo = new YoloV8(modelSelector);
        }

        public IEnumerable<List<Vector3>> CalculatePosesFromImage(Stream imageStream)
        {
            ImageSelector imageSelector = new ImageSelector(imageStream);
            IPoseResult result = yolo.Pose(imageSelector);

            return result.Boxes
                .Select(poseBoundingBox => poseBoundingBox.Keypoints
                    .Select(kp => new Vector3(kp.Point.X, kp.Point.Y, kp.Confidence))
                    .ToList()).ToList();
        }
    }
}