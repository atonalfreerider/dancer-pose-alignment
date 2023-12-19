using System.Numerics;
using Aurio;
using Aurio.FFmpeg;
using Aurio.FFT;
using Aurio.Matching;
using Aurio.Project;
using Aurio.Resampler;
using Aurio.TaskMonitor;
using Avalonia;
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
    Tuple<int, int> selectedPoseAndJointAtCamera = new(-1, -1);

    readonly Dictionary<string, VideoCapture> videoFiles = [];
    readonly Dictionary<string, double> videoFrameRates = [];
    Dictionary<string, double> videoOffsets = [];
    readonly Dictionary<string, Image> frameImages = [];
    readonly Dictionary<string, Image> graphicsImages = [];
    readonly Dictionary<int, string> indexToVideoFilePath = [];

    double timeFromStart = 0;
    double highestPositiveOffsetSeconds = 0;

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
        string? videoDirectory = VideoInputPath.Text;
        if (string.IsNullOrEmpty(videoDirectory)) return;
        if (!videoDirectory.EndsWith('/') && !videoDirectory.EndsWith('\\'))
        {
            videoDirectory += '/';
        }

        if (!Directory.Exists(videoDirectory)) return;

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
            AudioTrack audioTrack = new AudioTrack(new FileInfo(videoPath));
            audioTracks.Add(audioTrack);
        }

        HaitsmaKalkerFingerprintingModel model = new HaitsmaKalkerFingerprintingModel();
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
            VideoCapture videoCapture = new VideoCapture(videoFilePath);
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

            Size size = new Size(frameMat.Width, frameMat.Height);

            // add buttons to select camera role
            RadioButton radioButton = new RadioButton
            {
                Name = videoFilePath,
                GroupName = "Role",
                Content = Path.GetFileNameWithoutExtension(videoFilePath),
                Margin = new Thickness(5)
            };

            DynamicRadioButtonsPanel.Children.Add(radioButton);

            // add frame image to canvas
            Image frameImage = new Image
            {
                Width = size.Width,
                Height = size.Height,
                Source = frame
            };
            frameImages.Add(videoFilePath, frameImage);

            Canvas canvas = new Canvas
            {
                Width = size.Width,
                Height = size.Height
            };

            canvas.PointerPressed += Canvas_PointerPressed;
            canvas.PointerReleased += Canvas_PointerReleased;
            canvas.Children.Add(frameImage);

            // get yolo pose and draw
            cameraPoseSolver.CreateAndPlaceCamera(
                videoFilePath,
                new Vector2((float)size.Width, (float)size.Height),
                framesAt30Fps);

            cameraPoseSolver.SetPoseFromImage(frameMat.ToMemoryStream(), videoFilePath);

            DrawingImage drawingImage = new DrawingImage();
            Image poseDrawingImage = new Image
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
                
        cameraPoseSolver.Calculate3DPosesAndTotalError();
        foreach (string videoFilesKey in videoFiles.Keys)
        {
            RedrawCamera(videoFilesKey);
        }
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

            cameraPoseSolver.SetPoseFromImage(frameMat.ToMemoryStream(), videoFilePath);

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

        cameraPoseSolver.TryHomeCamera(selectedCamera);
        for (int i = 0; i < 50; i++)
        {
            cameraPoseSolver.CameraCircle();
        }

        cameraPoseSolver.IterationLoop();

        if (cameraPoseSolver.AreAllCamerasOriented())
        {
            cameraPoseSolver.Calculate3DPosesAndTotalError();
        }
        
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
        if (selectedPoseAndJointAtCamera.Item1 > -1 && selectedPoseAndJointAtCamera.Item2 > -1 &&
            GetSelectedButton() == "Move")
        {
            cameraPoseSolver.MoveKeypointAtCam(selectedCamera, new Vector2((float)x, (float)y),
                selectedPoseAndJointAtCamera);
            RedrawCamera(selectedCamera);
        }

        selectedPoseAndJointAtCamera = new Tuple<int, int>(-1, -1);
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
        Tuple<int, int> leadAndFollowIndex = cameraPoseSolver.LeadAndFollowIndicesAtCameraAtFrame(camName);

        List<Vector2> originCross = cameraPoseSolver.ReverseProjectOriginCrossAtCamera(camName);
        List<Vector2> leadProjectionsAtFrame = cameraPoseSolver.ReverseProjectionOfPoseAtCamera(camName, true);
        List<Vector2> followProjectionsAtFrame = cameraPoseSolver.ReverseProjectionOfPoseAtCamera(camName, false);

        List<Tuple<Vector2, Vector2>> cameraPositions2D = cameraPoseSolver.ReverseProjectCameraPositionsAtCameraAndManualPair(camName);

        DrawingImage drawingImage = new DrawingImage();
        DrawingGroup drawingGroup = PreviewDrawer.DrawGeometry(
            cameraPoseSolver.PosesAtFrameAtCamera(camName),
            new Size(graphicsImages[camName].Width, graphicsImages[camName].Height),
            leadAndFollowIndex.Item1,
            leadAndFollowIndex.Item2,
            PoseType.Coco,
            originCross,
            leadProjectionsAtFrame,
            followProjectionsAtFrame,
            cameraPositions2D);
        drawingImage.Drawing = drawingGroup;

        graphicsImages[camName].Source = drawingImage;
    }

    #region PERSPECTIVE

    const float TranslateStepSize = .1f;
    const float AngleStepSize = .01f;

    void YawLeftButton_Click(object sender, RoutedEventArgs e)
    {
        if (string.IsNullOrEmpty(selectedCamera)) return;
        cameraPoseSolver.YawCamera(selectedCamera, -AngleStepSize);
        RedrawCamera(selectedCamera);
    }

    void YawRightButton_Click(object sender, RoutedEventArgs e)
    {
        if (string.IsNullOrEmpty(selectedCamera)) return;
        cameraPoseSolver.YawCamera(selectedCamera, AngleStepSize);
        RedrawCamera(selectedCamera);
    }

    void PitchUpButton_Click(object sender, RoutedEventArgs e)
    {
        if (string.IsNullOrEmpty(selectedCamera)) return;
        cameraPoseSolver.PitchCamera(selectedCamera, -AngleStepSize);
        RedrawCamera(selectedCamera);
    }

    void PitchDownButton_Click(object sender, RoutedEventArgs e)
    {
        if (string.IsNullOrEmpty(selectedCamera)) return;
        cameraPoseSolver.PitchCamera(selectedCamera, AngleStepSize);
        RedrawCamera(selectedCamera);
    }

    void ZoomInButton_Click(object sender, RoutedEventArgs e)
    {
        if (string.IsNullOrEmpty(selectedCamera)) return;
        cameraPoseSolver.ZoomCamera(selectedCamera, AngleStepSize);
        RedrawCamera(selectedCamera);
    }

    void ZoomOutButton_Click(object sender, RoutedEventArgs e)
    {
        if (string.IsNullOrEmpty(selectedCamera)) return;
        cameraPoseSolver.ZoomCamera(selectedCamera, -AngleStepSize);
        RedrawCamera(selectedCamera);
    }

    void RollLeftButton_Click(object sender, RoutedEventArgs e)
    {
        if (string.IsNullOrEmpty(selectedCamera)) return;
        cameraPoseSolver.RollCamera(selectedCamera, AngleStepSize);
        RedrawCamera(selectedCamera);
    }

    void RollRightButton_Click(object sender, RoutedEventArgs e)
    {
        if (string.IsNullOrEmpty(selectedCamera)) return;
        cameraPoseSolver.RollCamera(selectedCamera, -AngleStepSize);
        RedrawCamera(selectedCamera);
    }

    void TranslateLeftButton_Click(object sender, RoutedEventArgs e)
    {
        if (string.IsNullOrEmpty(selectedCamera)) return;
        cameraPoseSolver.MoveCameraRight(selectedCamera, -TranslateStepSize);
        RedrawCamera(selectedCamera);
    }

    void TranslateRightButton_Click(object sender, RoutedEventArgs e)
    {
        if (string.IsNullOrEmpty(selectedCamera)) return;
        cameraPoseSolver.MoveCameraRight(selectedCamera, TranslateStepSize);
        RedrawCamera(selectedCamera);
    }

    void TranslateUpButton_Click(object sender, RoutedEventArgs e)
    {
        if (string.IsNullOrEmpty(selectedCamera)) return;
        cameraPoseSolver.MoveCameraUp(selectedCamera, TranslateStepSize);
        RedrawCamera(selectedCamera);
    }

    void TranslateDownButton_Click(object sender, RoutedEventArgs e)
    {
        if (string.IsNullOrEmpty(selectedCamera)) return;
        cameraPoseSolver.MoveCameraUp(selectedCamera, -TranslateStepSize);
        RedrawCamera(selectedCamera);
    }

    void TranslateForwardButton_Click(object sender, RoutedEventArgs e)
    {
        if (string.IsNullOrEmpty(selectedCamera)) return;
        cameraPoseSolver.MoveCameraForward(selectedCamera, TranslateStepSize);
        RedrawCamera(selectedCamera);
    }

    void TranslateBackwardButton_Click(object sender, RoutedEventArgs e)
    {
        if (string.IsNullOrEmpty(selectedCamera)) return;
        cameraPoseSolver.MoveCameraForward(selectedCamera, -TranslateStepSize);
        RedrawCamera(selectedCamera);
    }

    #endregion

    void SolverNextFrameButton_Click(object sender, RoutedEventArgs e)
    {
        if (!cameraPoseSolver.Advance()) return;

        timeFromStart += 1d / 30d;

        SetPreviewsToFrame();
    }
    
    void Solve_Click(object sender, RoutedEventArgs e)
    {
        cameraPoseSolver.IterationLoop();
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
            cameraPoseSolver.IterationLoop();
        }

        SetPreviewsToFrame();
    }

    void Save3D_Click(object sender, RoutedEventArgs e)
    {
        cameraPoseSolver.SaveData(VideoInputPath.Text);
    }
}