using System.Numerics;
using Avalonia;
using Avalonia.Controls;
using Avalonia.Input;
using Avalonia.Interactivity;
using Avalonia.Media;
using Avalonia.Media.Imaging;
using Avalonia.Threading;
using dancer_pose_alignment;
using Newtonsoft.Json;
using OpenCvSharp;
using Size = Avalonia.Size;
using Window = Avalonia.Controls.Window;

namespace GUI;

public partial class MainWindow : Window
{
    CameraPoseSolver cameraPoseSolver;
    int selectedCanvas = -1;
    int numCameras = 0;

    readonly List<VideoCapture> videoFiles = [];
    readonly List<double> videoFrameRates = [];
    readonly List<Image> frameImages = [];
    readonly List<Image> graphicsImages = [];
    
    double timeFromStart = 0;

    public MainWindow()
    {
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

        List<string> videoFilePaths = Directory.EnumerateFiles(videoDirectory, "*.*", SearchOption.TopDirectoryOnly)
            .Where(file => file.EndsWith(".mp4") || file.EndsWith(".avi") || file.EndsWith(".mkv")).ToList();
        numCameras = videoFilePaths.Count;

        string jsonContent = File.ReadAllText(Path.Combine(videoDirectory, "camera-positions.json"));
        List<Vector3> cameraPositions = JsonConvert.DeserializeObject<List<Vector3>>(jsonContent);

        // rescale and offset the camera Z and X, so that the origin is in the middle of the 10m x 8m dance floor. Also 1000 pixels = 10M
        for (int i = 0; i < cameraPositions.Count; i++)
        {
            cameraPositions[i] = cameraPositions[i] with
            {
                X = cameraPositions[i].X / 100 - 5,
                Z = -(cameraPositions[i].Z / 100 - 4) // orient forward
            };
        }

        cameraPoseSolver = new CameraPoseSolver(PoseType.Coco);

        videoFiles.Clear();
        frameImages.Clear();
        graphicsImages.Clear();
        CanvasContainer.Items.Clear();

        // SET FRAME ZERO FOR EACH CAMERA
        int camCount = 0;
        foreach (string videoFilePath in videoFilePaths)
        {
            // initialize video capture
            VideoCapture videoCapture = new VideoCapture(videoFilePath);
            videoFiles.Add(videoCapture);
            
            int frameCount = (int)videoCapture.Get(VideoCaptureProperties.FrameCount);
            double fps = videoCapture.Get(VideoCaptureProperties.Fps);
            videoFrameRates.Add(fps);
            double duration = frameCount / fps;
            
            int framesAt30Fps = (int)(duration * 30);
            
            if (cameraPoseSolver.MaximumFrameCount > framesAt30Fps)
            {
                cameraPoseSolver.MaximumFrameCount = framesAt30Fps;
            }

            videoCapture.Set(VideoCaptureProperties.PosFrames, 0);

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
                GroupName = "Role",
                Content = $"Camera{camCount}",
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
            frameImages.Add(frameImage);
            
            Canvas canvas = new Canvas
            {
                Width = size.Width,
                Height = size.Height
            };
            canvas.PointerPressed += Canvas_PointerPressed;
            canvas.Children.Add(frameImage);
            
            // get yolo pose and draw
            cameraPoseSolver.CreateAndPlaceCamera(
                new Vector2((float)size.Width, (float)size.Height),
                framesAt30Fps,
                cameraPositions[camCount]);
            
            cameraPoseSolver.PoseFromImage(frameMat.ToMemoryStream(), camCount);
            
            DrawingImage drawingImage = new DrawingImage();
            DrawingGroup drawingGroup = PreviewDrawer.DrawGeometry(
                cameraPoseSolver.PosesAtFrameAtCamera(camCount),
                size,
                -1,
                -1,
                PoseType.Coco);
            drawingImage.Drawing = drawingGroup;

            Image poseDrawingImage = new Image
            {
                Width = size.Width,
                Height = size.Height,
                Source = drawingImage
            };

            graphicsImages.Add(poseDrawingImage);
            canvas.Children.Add(poseDrawingImage);

            CanvasContainer.Items.Add(canvas);
            
            camCount++;
        }
    }

    void SetPreviewsToFrame()
    {
        for (int i = 0; i < numCameras; i++)
        {
            VideoCapture videoCapture = videoFiles[i];
            videoCapture.Set(VideoCaptureProperties.PosFrames, (int)(timeFromStart * videoFrameRates[i]));
            
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
            
            frameImages[i].Source = frame;
            
            cameraPoseSolver.PoseFromImage(frameMat.ToMemoryStream(), i);
            
            DrawingImage drawingImage = new DrawingImage();
            DrawingGroup drawingGroup = PreviewDrawer.DrawGeometry(
                cameraPoseSolver.PosesAtFrameAtCamera(i),
                new Size(frameMat.Width, frameMat.Height),
                -1,
                -1,
                PoseType.Coco);
            drawingImage.Drawing = drawingGroup;
            
            graphicsImages[i].Source = drawingImage;
        }
    }

    void Canvas_PointerPressed(object sender, PointerPressedEventArgs args)
    {
        // Mark this canvas as selected
        selectedCanvas = CanvasContainer.Items.IndexOf(sender as Canvas);
        
        PointerPoint point = args.GetCurrentPoint(sender as Control);
        
        double x = point.Position.X;
        double y = point.Position.Y;

        if (point.Properties.IsLeftButtonPressed)
        {
            SetDancer(new Vector2((float)x, (float)y), selectedCanvas);
        }

        if (cameraPoseSolver.TryHomeAllCameras())
        {
            cameraPoseSolver.Calculate3DPosesAndTotalError();
            for (int i = 0; i < numCameras; i++)
            {
                RedrawCamera(i);
            }
        }
    }
    
    string GetSelectedButton()
    {
        foreach (Control? child in DynamicRadioButtonsPanel.Children)
        {
            if (child is RadioButton { IsChecked: true } radioButton)
            {
                return radioButton.Content.ToString();
            }
        }

        return "0";
    }
    
    void SetDancer(Vector2 position, int camIndex)
    {
        string selectedButton = GetSelectedButton();
        cameraPoseSolver.MarkDancerAtCam(
            camIndex, 
            position, 
            selectedButton);
        
        RedrawCamera(camIndex);
    }

    void RedrawCamera(int camIndex)
    {
        Tuple<int, int> leadAndFollowIndex = cameraPoseSolver.LeadAndFollowIndicesAtCameraAtFrame(camIndex);

        List<Vector2> leadProjectionsAtFrame = cameraPoseSolver.ReverseProjectionOfLeadPoseAtCamera(camIndex);
        List<Vector2> followProjectionsAtFrame = cameraPoseSolver.ReverseProjectionOfFollowPoseAtCamera(camIndex);
        List<Vector2> originCross = cameraPoseSolver.ReverseProjectOriginCrossAtCamera(camIndex);
        
        DrawingImage drawingImage = new DrawingImage();
        DrawingGroup drawingGroup = PreviewDrawer.DrawGeometry(
            cameraPoseSolver.PosesAtFrameAtCamera(camIndex),
            new Size(graphicsImages[camIndex].Width, graphicsImages[camIndex].Height),
            leadAndFollowIndex.Item1,
            leadAndFollowIndex.Item2,
            PoseType.Coco,
            originCross,
            leadProjectionsAtFrame,
            followProjectionsAtFrame);
        drawingImage.Drawing = drawingGroup;
        
        graphicsImages[camIndex].Source = drawingImage;
    }
    

    #region PERSPECTIVE

    void YawLeftButton_Click(object sender, RoutedEventArgs e)
    {
        if (selectedCanvas == -1) return;
        cameraPoseSolver.YawCamera(selectedCanvas, -.01f);
        SetPreviewsToFrame();
    }

    void YawRightButton_Click(object sender, RoutedEventArgs e)
    {
        if (selectedCanvas == -1) return;
        cameraPoseSolver.YawCamera(selectedCanvas, .01f);
        SetPreviewsToFrame();
    }

    void PitchUpButton_Click(object sender, RoutedEventArgs e)
    {
        if (selectedCanvas == -1) return;
        cameraPoseSolver.PitchCamera(selectedCanvas, -.01f);
        SetPreviewsToFrame();
    }

    void PitchDownButton_Click(object sender, RoutedEventArgs e)
    {
        if (selectedCanvas == -1) return;
        cameraPoseSolver.PitchCamera(selectedCanvas, .01f);
        SetPreviewsToFrame();
    }

    void ZoomInButton_Click(object sender, RoutedEventArgs e)
    {
        if (selectedCanvas == -1) return;
        cameraPoseSolver.ZoomCamera(selectedCanvas, .01f);
        SetPreviewsToFrame();
    }

    void ZoomOutButton_Click(object sender, RoutedEventArgs e)
    {
        if (selectedCanvas == -1) return;
        cameraPoseSolver.ZoomCamera(selectedCanvas, -.01f);
        SetPreviewsToFrame();
    }

    void RollLeftButton_Click(object sender, RoutedEventArgs e)
    {
        if (selectedCanvas == -1) return;
        cameraPoseSolver.RollCamera(selectedCanvas, .01f);
        SetPreviewsToFrame();
    }

    void RollRightButton_Click(object sender, RoutedEventArgs e)
    {
        if (selectedCanvas == -1) return;
        cameraPoseSolver.RollCamera(selectedCanvas, -.01f);
        SetPreviewsToFrame();
    }

    void TranslateLeftButton_Click(object sender, RoutedEventArgs e)
    {
        if (selectedCanvas == -1) return;
        cameraPoseSolver.MoveCameraRight(selectedCanvas, -.01f);
        SetPreviewsToFrame();
    }

    void TranslateRightButton_Click(object sender, RoutedEventArgs e)
    {
        if (selectedCanvas == -1) return;
        cameraPoseSolver.MoveCameraRight(selectedCanvas, .01f);
        SetPreviewsToFrame();
    }

    void TranslateUpButton_Click(object sender, RoutedEventArgs e)
    {
        if (selectedCanvas == -1) return;
        cameraPoseSolver.MoveCameraUp(selectedCanvas, .01f);
        SetPreviewsToFrame();
    }

    void TranslateDownButton_Click(object sender, RoutedEventArgs e)
    {
        if (selectedCanvas == -1) return;
        cameraPoseSolver.MoveCameraUp(selectedCanvas, -.01f);
        SetPreviewsToFrame();
    }

    void TranslateForwardButton_Click(object sender, RoutedEventArgs e)
    {
        if (selectedCanvas == -1) return;
        cameraPoseSolver.MoveCameraForward(selectedCanvas, -.01f);
        SetPreviewsToFrame();
    }

    void TranslateBackwardButton_Click(object sender, RoutedEventArgs e)
    {
        if (selectedCanvas == -1) return;
        cameraPoseSolver.MoveCameraForward(selectedCanvas, -.01f);
        SetPreviewsToFrame();
    }

    #endregion

    void SolverNextFrameButton_Click(object sender, RoutedEventArgs e)
    {
        if (!cameraPoseSolver.Advance()) return;
        
        timeFromStart += 1d / 30d;
        
        cameraPoseSolver.IterationLoop();
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
            cameraPoseSolver.IterationLoop();
        }

        SetPreviewsToFrame();
    }

    void Save3D_Click(object sender, RoutedEventArgs e)
    {
        cameraPoseSolver.SaveData(VideoInputPath.Text);
    }
}