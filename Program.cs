using System.CommandLine;
using System.CommandLine.NamingConventionBinder;
using System.Numerics;
using Compunet.YoloV8;
using Compunet.YoloV8.Data;
using OpenCvSharp;

namespace dancer_pose_alignment;

static class Program
{
    class Args
    {
        public string InputPath { get; set; }
        public string OutputDb { get; set; }
    }

    static void Main(string[] args)
    {
        RootCommand rootCommand = new()
        {
            new Argument<string>("InputPath"),

            new Argument<string>("OutputDb")
        };

        rootCommand.Description = "Merge dance poses into a single database with animations between frames";

        // Note that the parameters of the handler method are matched according to the names of the options 
        rootCommand.Handler = CommandHandler.Create<Args>(Parse);

        rootCommand.Invoke(args);

        Environment.Exit(0);
    }

    static void Parse(Args args)
    {
        ModelSelector modelSelector = new ModelSelector("yolov8x-pose.onnx");
        YoloV8 yolo = new(modelSelector);

        int frameCount = 0;
        List<List<Tuple<IPoseBoundingBox, IPoseBoundingBox>>> allCameras = new();
        foreach (string directory in Directory.EnumerateDirectories(args.InputPath))
        {
            int fileCount = Directory.EnumerateFiles(directory).Count();
            if (fileCount > frameCount)
            {
                frameCount = fileCount;
            }
            
            Dancer leadForCam = new Dancer()
            {
                Role = Role.Lead
            };
            Dancer followForCam = new Dancer()
            {
                Role = Role.Follow
            };  

            List<Tuple<IPoseBoundingBox, IPoseBoundingBox>> cameraFrames = new();
            allCameras.Add(cameraFrames);
            // iterate through camera frames
            foreach (string filePath in Directory.EnumerateFiles(directory))
            {
                Console.WriteLine(filePath);
                ImageSelector imageSelector = new ImageSelector(filePath);
                IPoseResult result = yolo.Pose(imageSelector);

                int tallest = 0;
                IPoseBoundingBox tallestBox = null;
                int secondTallest = 0;
                IPoseBoundingBox secondTallestBox = null;

                foreach (IPoseBoundingBox poseBoundingBox in result.Boxes)
                {
                    int height = poseBoundingBox.Bounds.Height;
                    if (height > tallest)
                    {
                        secondTallest = tallest;
                        secondTallestBox = tallestBox;
                        tallest = height;
                        tallestBox = poseBoundingBox;
                    }
                    else if (height > secondTallest)
                    {
                        secondTallest = height;
                        secondTallestBox = poseBoundingBox;
                    }
                }

                Tuple<IPoseBoundingBox, IPoseBoundingBox> leadAndFollow = new(tallestBox, secondTallestBox);
                cameraFrames.Add(leadAndFollow);
            }
        }

        Dancer lead = new()
        {
            Role = Role.Lead
        };
        Dancer follow = new Dancer()
        {
            Role = Role.Follow
        };
        for (int i = 0; i < frameCount; i++)
        {
            Console.WriteLine(i);
            Tuple<IPoseBoundingBox, IPoseBoundingBox> leadAndFollowACam = allCameras[0][i];
            Tuple<IPoseBoundingBox, IPoseBoundingBox> leadAndFollowBCam = allCameras[1][i];
            List<Point2d> leadA =
                leadAndFollowACam.Item1.Keypoints.Select(x => new Point2d(x.Point.X, x.Point.Y)).ToList();
            List<Point2d> leadB =
                leadAndFollowBCam.Item1.Keypoints.Select(x => new Point2d(x.Point.X, x.Point.Y)).ToList();

            Mat hCv0 = Cv2.FindHomography(leadA, leadA);
            Mat camPose0 = CameraPoseFromHomography(hCv0);
            
            Mat hCv1 = Cv2.FindHomography(leadA, leadB);
            Mat camPose1 = CameraPoseFromHomography(hCv1);
            
            OutputArray outputArray = OutputArray.Create(new Mat());
            Cv2.TriangulatePoints(
                InputArray.Create(camPose0),
                InputArray.Create(camPose1),
                InputArray.Create(leadA),
                InputArray.Create(leadB),
                outputArray);

            double[,] output = ConvertMatToDoubleArray(outputArray.GetMat());

            List<Vector3> leadPose = ToCartesian(output);
            lead.PosesByFrame.Add(leadPose);
        }

        SqliteOutput sqliteOutput = new SqliteOutput(args.OutputDb, frameCount);
        sqliteOutput.Serialize(new Tuple<Dancer, Dancer>(lead, follow));
    }
    
    static Mat CameraPoseFromHomography(Mat H)
    {
        Mat pose = Mat.Eye(3, 4, MatType.CV_64FC1);

        double norm1 = Cv2.Norm(H.Col(0));
        double norm2 = Cv2.Norm(H.Col(1));
        double tnorm = (norm1 + norm2) / 2.0;

        Mat v1 = H.Col(0);
        Mat v2 = pose.Col(0);
        Cv2.Normalize(v1, v2);

        v1 = H.Col(1);
        v2 = pose.Col(1);
        Cv2.Normalize(v1, v2);

        v1 = pose.Col(0);
        v2 = pose.Col(1);
        Mat v3 = v1.Cross(v2);
        Mat c2 = pose.Col(2);
        v3.CopyTo(c2);

        // Fix for the last line
        Mat t = H.Col(2) / tnorm;
        t.CopyTo(pose.Col(3));

        return pose;
    }

    static double[,] ConvertMatToDoubleArray(Mat mat)
    {
        // Ensure the Mat type is CV_64F (double)
        if (mat.Type() != MatType.CV_64F)
        {
            throw new ArgumentException("The provided Mat object must be of type CV_64F (double).");
        }

        int rows = mat.Rows;
        int cols = mat.Cols;

        double[,] result = new double[rows, cols];

        // Copy data from Mat to the double array
        for (int i = 0; i < rows; i++)
        {
            for (int j = 0; j < cols; j++)
            {
                result[i, j] = mat.At<double>(i, j);
            }
        }

        return result;
    }
    
    static List<Vector3> ToCartesian(double[,] values)
    {
        if (values.GetLength(0) != 4)
        {
            throw new ArgumentException("Expected a 4 x n matrix", nameof(values));
        }

        List<Vector3> result = new List<Vector3>();

        for (int i = 0; i < values.GetLength(1); i++)
        {
            double w = values[3, i];
            if (Math.Abs(w) < double.Epsilon)
            {
                Console.WriteLine("W value is too close to zero.");
            }

            float x = (float)(values[0, i] / w);
            float y = (float)(values[1, i] / w);
            float z = (float)(values[2, i] / w);

            result.Add(new Vector3(x, y, z));
        }

        return result;
    }
}