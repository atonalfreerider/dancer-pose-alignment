using System.Numerics;
using Aurio;
using Aurio.FFmpeg;
using Aurio.FFT;
using Aurio.Matching;
using Aurio.Project;
using Aurio.Resampler;
using Aurio.TaskMonitor;
using Avalonia.Controls;
using Avalonia.Input;
using Avalonia.Interactivity;
using Avalonia.Media;
using Avalonia.Media.Imaging;
using dancer_pose_alignment;
using Newtonsoft.Json;
using OpenCvSharp;
using Size = Avalonia.Size;
using Window = Avalonia.Controls.Window;

namespace GUI;

public partial class MainWindow : Window
{
    CameraPoseSolver cameraPoseSolver;
    string selectedCamera = "";
    Tuple<PoseBoundingBox?, int> selectedPoseAndJointAtCamera = new(null, -1);

    readonly Dictionary<string, VideoCapture> videoFiles = [];
    readonly Dictionary<string, double> videoFrameRates = [];
    Dictionary<string, double> videoOffsets = [];
    readonly Dictionary<string, Image> frameImages = [];
    readonly Dictionary<string, Image> graphicsImages = [];
    readonly Dictionary<int, string> indexToVideoFilePath = [];

    double timeFromStart = 0;
    double highestPositiveOffsetSeconds = 0;

    string dbPath;

    public MainWindow()
    {
        // clip alignment

        // Use PFFFT as FFT implementation
        FFTFactory.Factory = new Aurio.PFFFT.FFTFactory();
        // Use Soxr as resampler implementation
        ResamplerFactory.Factory = new Aurio.Soxr.ResamplerFactory();
        // Use FFmpeg for file reading/decoding
        AudioStreamFactory.AddFactory(new FFmpegAudioStreamFactory());

        InitializeComponent();
    }

    void LoadVideosButton_Click(object sender, RoutedEventArgs e)
    {
        string videoDirectory = VideoDirectory();
        dbPath = Path.Combine(videoDirectory, "larissa-kadu-recap.db");

        if (string.IsNullOrEmpty(videoDirectory) || !Directory.Exists(videoDirectory)) return;

        if (File.Exists(Path.Combine(videoDirectory, "camera-time-offsets.json")))
        {
            videoOffsets = JsonConvert.DeserializeObject<Dictionary<string, double>>(
                File.ReadAllText(Path.Combine(videoDirectory, "camera-time-offsets.json")));

            LoadVideos(videoOffsets);
            return;
        }

        List<AudioTrack> audioTracks = [];
        foreach (string videoPath in Directory.EnumerateFiles(videoDirectory, "*.mp4"))
        {
            AudioTrack audioTrack = new(new FileInfo(videoPath));
            audioTracks.Add(audioTrack);
        }

        HaitsmaKalkerFingerprintingModel model = new();
        model.FingerprintingFinished += delegate
        {
            model.FindAllMatches(
                new ProgressMonitor(),
                TrackTimingCallback
            );
        };

        model.Reset();
        model.Fingerprint(audioTracks, new ProgressMonitor());
    }

    void TrackTimingCallback(List<Match> matches)
    {
        Dictionary<string, Dictionary<string, List<double>>> trackAndOffsetsToOtherTracks = [];

        foreach (Match match in matches)
        {
            string trackName = match.Track1.FileInfo.FullName;
            string otherTrackName = match.Track2.FileInfo.FullName;
            double offset = match.Offset.TotalSeconds;
            trackAndOffsetsToOtherTracks.TryGetValue(trackName, out Dictionary<string, List<double>>? otherTracks);
            if (otherTracks == null)
            {
                otherTracks = new Dictionary<string, List<double>>();
                trackAndOffsetsToOtherTracks.Add(trackName, otherTracks);
            }

            otherTracks.TryGetValue(otherTrackName, out List<double>? offsets);
            if (offsets == null)
            {
                offsets = new List<double>();
                otherTracks.Add(otherTrackName, offsets);
            }

            offsets.Add(offset);
        }

        // get the median offset, incrementing from the first entry to the last
        KeyValuePair<string, Dictionary<string, List<double>>> maxKeyValuePair =
            trackAndOffsetsToOtherTracks.MaxBy(kv =>
                kv.Value.Count); // Take the first item after ordering, which will have the maximum count

        videoOffsets.Add(maxKeyValuePair.Key, 0);

        foreach ((string other, List<double> offsets) in maxKeyValuePair.Value)
        {
            offsets.Sort();
            double medianOffset = offsets[offsets.Count / 2];
            videoOffsets.Add(other, medianOffset);
        }

        File.WriteAllText(
            Path.Combine(VideoInputPath.Text, "camera-time-offsets.json"),
            JsonConvert.SerializeObject(videoOffsets, Formatting.Indented));

        LoadVideos(videoOffsets);
    }

    void LoadVideos(Dictionary<string, double> videoFilePathsAndOffsets)
    {
        string videoDirectory = VideoDirectory();
        string dbPath = Path.Combine(videoDirectory, "larissa-kadu-recap.db");
        string affineDirectory = Path.Combine(videoDirectory, "affine/");
        
        cameraPoseSolver = new CameraPoseSolver(PoseType.Coco);

        videoFiles.Clear();
        frameImages.Clear();
        graphicsImages.Clear();
        CanvasContainer.Items.Clear();

        highestPositiveOffsetSeconds = videoFilePathsAndOffsets.Max(kvp => kvp.Value);

        // SET FRAME ZERO FOR EACH CAMERA
        int camCount = 0;
        foreach ((string videoFilePath, double myOffset) in videoFilePathsAndOffsets)
        {
            // initialize video capture
            VideoCapture videoCapture = new(videoFilePath);
            videoFiles.Add(videoFilePath, videoCapture);
            indexToVideoFilePath.Add(camCount, videoFilePath);

            int frameCount = (int)videoCapture.Get(VideoCaptureProperties.FrameCount);
            double fps = videoCapture.Get(VideoCaptureProperties.Fps);
            videoFrameRates.Add(videoFilePath, fps);
            double duration = frameCount / fps;

            int framesAt30Fps = (int)(duration * 30);

            if (cameraPoseSolver.MaximumFrameCount > framesAt30Fps)
            {
                cameraPoseSolver.MaximumFrameCount = framesAt30Fps;
            }

            int startingFrame = (int)((highestPositiveOffsetSeconds - myOffset) * fps);

            videoCapture.Set(VideoCaptureProperties.PosFrames, startingFrame);

            OutputArray outputArray = new Mat();
            videoCapture.Read(outputArray);

            Mat frameMat = outputArray.GetMat();

            // render image
            Bitmap frame;
            try
            {
                frame = Bitmap.DecodeToWidth(frameMat.ToMemoryStream(), frameMat.Width);
            }
            catch (OpenCVException openCvException)
            {
                // end of video
                Console.WriteLine(openCvException.Message);
                continue;
            }

            Size size = new(frameMat.Width, frameMat.Height);

            // add frame image to canvas
            Image frameImage = new()
            {
                Width = size.Width,
                Height = size.Height,
                Source = frame
            };
            frameImages.Add(videoFilePath, frameImage);

            Canvas canvas = new()
            {
                Width = size.Width,
                Height = size.Height
            };

            canvas.PointerPressed += Canvas_PointerPressed;
            canvas.PointerReleased += Canvas_PointerReleased;
            canvas.Children.Add(frameImage);

            // get yolo pose and draw
            cameraPoseSolver.CreateCamera(
                videoFilePath,
                new Vector2((float)size.Width, (float)size.Height),
                framesAt30Fps,
                startingFrame,
                frameCount,
                dbPath);
            
            string fileName = Path.GetFileNameWithoutExtension(videoFilePath);
            cameraPoseSolver.SetPoseFromImage(videoFilePath); 
           
            string affinePath = Path.Combine(affineDirectory, fileName + ".mp4.json");
            // if pre-cached json, load it
            if (File.Exists(affinePath))
            {
                List<Vector3> affineTransform = JsonConvert.DeserializeObject<List<Vector3>>(
                    File.ReadAllText(affinePath));
                cameraPoseSolver.SetAllAffine(affineTransform, videoFilePath);
            }
            else
            {
                // do nothing
            }

            DrawingImage drawingImage = new();
            Image poseDrawingImage = new()
            {
                Width = size.Width,
                Height = size.Height,
                Source = drawingImage
            };

            graphicsImages.Add(videoFilePath, poseDrawingImage);
            canvas.Children.Add(poseDrawingImage);

            CanvasContainer.Items.Add(canvas);

            camCount++;
        }
        
        cameraPoseSolver.HomeAllCameras();
        //cameraPoseSolver.SetCamR();
        //cameraPoseSolver.HomeAllCameras();
        RecalculateAndRedraw();
    }

    void SetPreviewsToFrame()
    {
        foreach ((string videoFilePath, VideoCapture videoCapture) in videoFiles)
        {
            videoCapture.Set(
                VideoCaptureProperties.PosFrames,
                (int)((highestPositiveOffsetSeconds - videoOffsets[videoFilePath] + timeFromStart)
                      * videoFrameRates[videoFilePath]));

            OutputArray outputArray = new Mat();
            videoCapture.Read(outputArray);

            Mat frameMat = outputArray.GetMat();

            // render frame
            Bitmap frame;
            try
            {
                frame = Bitmap.DecodeToWidth(frameMat.ToMemoryStream(), frameMat.Width);
            }
            catch (OpenCVException openCvException)
            {
                // end of video
                Console.WriteLine(openCvException.Message);
                continue;
            }

            frameImages[videoFilePath].Source = frame;

            cameraPoseSolver.CalculateLeadFollow3DPoses();

            RedrawCamera(videoFilePath);
        }
    }

    void Canvas_PointerPressed(object sender, PointerPressedEventArgs args)
    {
        // Mark this canvas as selected
        selectedCamera = indexToVideoFilePath[CanvasContainer.Items.IndexOf(sender as Canvas)];

        PointerPoint point = args.GetCurrentPoint(sender as Control);

        double x = point.Position.X;
        double y = point.Position.Y;

        if (point.Properties.IsLeftButtonPressed)
        {
            SetDancer(new Vector2((float)x, (float)y), selectedCamera);
        }
        
        cameraPoseSolver.HomeAllCameras();
        //cameraPoseSolver.SetCamR();
        
        RecalculateAndRedraw();
    }

    void RecalculateAndRedraw()
    {
        cameraPoseSolver.CalculateLeadFollow3DPoses();
        
        foreach (string videoFilesKey in videoFiles.Keys)
        {
            RedrawCamera(videoFilesKey);
        }
    }

    void Canvas_PointerReleased(object sender, PointerReleasedEventArgs args)
    {
        // Mark this canvas as selected
        selectedCamera = indexToVideoFilePath[CanvasContainer.Items.IndexOf(sender as Canvas)];

        PointerPoint point = args.GetCurrentPoint(sender as Control);

        double x = point.Position.X;
        double y = point.Position.Y;
        if (selectedPoseAndJointAtCamera.Item1 != null && selectedPoseAndJointAtCamera.Item2 > -1 &&
            GetSelectedButton() == "Move")
        {
            cameraPoseSolver.MoveKeypointAtCam(
                selectedCamera,
                new Vector2((float)x, (float)y),
                selectedPoseAndJointAtCamera);
            RedrawCamera(selectedCamera);
        }

        selectedPoseAndJointAtCamera = new Tuple<PoseBoundingBox?, int>(null, -1);
    }

    string GetSelectedButton()
    {
        foreach (Control? child in DynamicRadioButtonsPanel.Children)
        {
            if (child is RadioButton { IsChecked: true } radioButton)
            {
                return radioButton.Name.ToString();
            }
        }

        return "0";
    }

    void SetDancer(Vector2 position, string camName)
    {
        string selectedButton = GetSelectedButton();
        selectedPoseAndJointAtCamera = cameraPoseSolver.MarkDancerAtCam(
            camName,
            position,
            selectedButton);
    }

    void RedrawCamera(string camName)
    {
        List<Vector2> originCross = cameraPoseSolver.ReverseProjectOriginCrossAtCamera(camName);
        List<Vector2> leadProjectionsAtFrame = cameraPoseSolver.ReverseProjectionOfPoseAtCamera(camName, true);
        List<Vector2> followProjectionsAtFrame = cameraPoseSolver.ReverseProjectionOfPoseAtCamera(camName, false);

        List<Vector2> cameraPositions2D = cameraPoseSolver.ReverseProjectCameraPositionsAtCameraAndManualPair(camName);

        DrawingImage drawingImage = new();
        DrawingGroup drawingGroup = PreviewDrawer.DrawGeometry(
            cameraPoseSolver.GetPosesAtFrameAtCamera(camName),
            new Size(graphicsImages[camName].Width, graphicsImages[camName].Height),
            PoseType.Coco,
            originCross,
            leadProjectionsAtFrame,
            followProjectionsAtFrame,
            cameraPositions2D);
        drawingImage.Drawing = drawingGroup;

        graphicsImages[camName].Source = drawingImage;
    }

    void SolverNextFrameButton_Click(object sender, RoutedEventArgs e)
    {
        if (!cameraPoseSolver.Advance()) return;

        timeFromStart += 1d / 30d;
        
        SetPreviewsToFrame();
    }

    void SolverPreviousFrameButton_Click(object sender, RoutedEventArgs e)
    {
        if (!cameraPoseSolver.Rewind()) return;

        timeFromStart -= 1d / 30d;

        SetPreviewsToFrame();
    }

    void RunUntilEnd_Click(object sender, RoutedEventArgs e)
    {
        while (cameraPoseSolver.Advance())
        {
            timeFromStart += 1d / 30d;
            cameraPoseSolver.CalculateLeadFollow3DPoses();
            if (!cameraPoseSolver.AreLeadAndFollowAssignedForFrame())
            {
                break;
            }
        }
        SetPreviewsToFrame();
    }

    void Save3D_Click(object sender, RoutedEventArgs e)
    {
        cameraPoseSolver.SaveData(VideoInputPath.Text);
    }

    string VideoDirectory()
    {
        string? videoDirectory = VideoInputPath.Text;
        if (string.IsNullOrEmpty(videoDirectory)) return "";
        if (!videoDirectory.EndsWith('/') && !videoDirectory.EndsWith('\\'))
        {
            videoDirectory += '/';
        }

        return videoDirectory;
    }
}