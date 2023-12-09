using System.Numerics;
using Avalonia;
using Avalonia.Controls;
using Avalonia.Controls.Shapes;
using Avalonia.Input;
using Avalonia.Interactivity;
using Avalonia.Media;
using Avalonia.Media.Imaging;
using Avalonia.Threading;
using dancer_pose_alignment;
using Newtonsoft.Json;
using OpenCvSharp;
using Path = System.IO.Path;
using Point = Avalonia.Point;
using Size = Avalonia.Size;
using Window = Avalonia.Controls.Window;

namespace GUI;

public partial class MainWindow : Window
{
    // initializeation
    VideoCapture frameSource;
    Dictionary<int, Dictionary<int, List<Vector3>>> alphaPosesByFrameByPerson;

    // indices for tracked figures
    int currentLeadIndex = -1;
    int mirrorCurrentLeadIndex = -1;
    int currentFollowIndex = -1;
    int mirrorCurrentFollowIndex = -1;

    // frame numbers
    int numCameras = 0;
    int frameCount = 0;
    int totalFrameCount = 0;

    // current state of frame
    Dictionary<int, List<Vector3>> posesByPersonAtFrame = new();
    readonly List<Tuple<int, bool>> currentSelectedCamerasAndPoseAnchor = [];
    readonly List<Tuple<int, bool>> mirrorCurrentSelectedCamerasAndPoseAnchor = [];

    // states to serialize
    // 0 - lead, 1 - follow, 2 - mirror lead, 3 - mirror follow. they are updated at every frame change
    readonly List<List<List<Vector3>>> finalDancerPoses = [];
    readonly List<List<Tuple<int, bool>>> finalIndexCamerasAndPoseAnchor = [];
    readonly List<List<Tuple<int, bool>>> finalIndexMirroredCamerasAndPoseAnchor = [];

    List<Vector3> cameraPositions = [];
    List<float> cameraFocalLengths = [];

    public MainWindow()
    {
        InitializeComponent();
        VideoFilesDropdown.SelectionChanged += delegate
        {
            // reset the state
            frameCount = 0;
            currentLeadIndex = -1;
            mirrorCurrentLeadIndex = -1;
            currentFollowIndex = -1;
            mirrorCurrentFollowIndex = -1;
            posesByPersonAtFrame = new Dictionary<int, List<Vector3>>();
            finalDancerPoses.Clear();
            finalIndexCamerasAndPoseAnchor.Clear();
            finalIndexMirroredCamerasAndPoseAnchor.Clear();
            ClearCameraMarkers();

            alphaPosesByFrameByPerson = AlphaPose.PosesByFrameByPerson(GetAlphaPoseJsonPath());
            totalFrameCount = FindMaxFrame();
            // populate final index lists
            for (int i = 0; i < totalFrameCount; i++)
            {
                List<List<Vector3>> frameDancerPoses = [];
                for (int j = 0; j < 4; j++)
                {
                    frameDancerPoses.Add([]); // 0 - lead, 1 - follow, 2 - mirror lead, 3 - mirror follow
                }

                finalDancerPoses.Add(frameDancerPoses);
                finalIndexCamerasAndPoseAnchor.Add([]);
                finalIndexMirroredCamerasAndPoseAnchor.Add([]);
            }

            frameSource = new VideoCapture(GetVideoPath());
            RenderZero();
        };
    }

    #region POSE REFINEMENT

    void LoadVideosButton_Click(object sender, RoutedEventArgs e)
    {
        string? videoDirectory = VideoInputPath.Text;
        if (string.IsNullOrEmpty(videoDirectory)) return;
        if (!videoDirectory.EndsWith('/') && !videoDirectory.EndsWith('\\'))
        {
            videoDirectory += '/';
        }

        if (Directory.Exists(videoDirectory))
        {
            List<string> videoFiles = Directory.EnumerateFiles(videoDirectory, "*.*", SearchOption.TopDirectoryOnly)
                .Where(file => file.EndsWith(".mp4") || file.EndsWith(".avi") || file.EndsWith(".mkv")).ToList();
            VideoFilesDropdown.ItemsSource = videoFiles;
            numCameras = videoFiles.Count;

            for (int i = 0; i < numCameras; i++)
            {
                RadioButton radioButton = new RadioButton
                {
                    GroupName = "Role",
                    Content = $"Camera{i}",
                    Margin = new Thickness(5)
                };

                DynamicRadioButtonsPanel.Children.Add(radioButton);

                // populate camera markers
                currentSelectedCamerasAndPoseAnchor.Add(new Tuple<int, bool>(-1, false));
                mirrorCurrentSelectedCamerasAndPoseAnchor.Add(new Tuple<int, bool>(-1, false));

                RadioButton layoutCamButton = new RadioButton { Content = $"Camera{i}" };
                RadioButtonsPanel.Children.Add(layoutCamButton);
            }
        }
    }

    void ClearDancers_Click(object sender, RoutedEventArgs e)
    {
        currentLeadIndex = -1;
        currentFollowIndex = -1;
        mirrorCurrentFollowIndex = -1;
        mirrorCurrentLeadIndex = -1;

        ClearCameraMarkers();
        RedrawPoses();
    }

    void ClearCameraMarkers()
    {
        for (int i = 0; i < numCameras; i++)
        {
            currentSelectedCamerasAndPoseAnchor[i] = new Tuple<int, bool>(-1, false);
            mirrorCurrentSelectedCamerasAndPoseAnchor[i] = new Tuple<int, bool>(-1, false);
        }
    }

    void BackButton_Click(object sender, RoutedEventArgs e)
    {
        if (frameCount <= 0) return;
        SaveIndicesForFrame();
        frameCount--;
        RenderFrame();
    }

    void NextFrameButton_Click(object sender, RoutedEventArgs e)
    {
        if (frameCount >= totalFrameCount) return;

        SaveIndicesForFrame();
        frameCount++;
        RenderFrame();
    }

    void RunUntilNext_Click(object sender, RoutedEventArgs e)
    {
        while (frameCount < totalFrameCount && Continuity())
        {
            SaveIndicesForFrame();
            frameCount++;
            RenderFrame();
        }
    }

    void SaveButton_Click(object sender, RoutedEventArgs e)
    {
        string saveDirectory = Environment.CurrentDirectory;
        string cameraName = "0";
        if (!string.IsNullOrEmpty(VideoInputPath.Text))
        {
            saveDirectory = Directory.GetParent(GetVideoPath()).FullName;
            cameraName = Path.GetFileNameWithoutExtension(GetVideoPath());
        }

        SaveTo(saveDirectory, cameraName);
    }

    #endregion

    #region DRAW INTERACTION

    void RenderZero()
    {
        frameSource.Set(VideoCaptureProperties.PosFrames, frameCount);

        OutputArray outputArray = new Mat();
        frameSource.Read(outputArray);

        Mat frameMat = outputArray.GetMat();
        VideoCanvas.Width = frameMat.Width;
        VideoCanvas.Height = frameMat.Height;

        RenderFrame();
    }

    void RenderFrame()
    {
        OutputArray outputArray = new Mat();
        frameSource.Set(VideoCaptureProperties.PosFrames, frameCount);
        frameSource.Read(outputArray);

        Mat frameMat = outputArray.GetMat();

        Bitmap frame;
        try
        {
            frame = Bitmap.DecodeToWidth(frameMat.ToMemoryStream(), frameMat.Width);
        }
        catch (OpenCVException e)
        {
            // end of video
            Console.WriteLine(e.Message);
            return;
        }

        posesByPersonAtFrame = new Dictionary<int, List<Vector3>>();
        foreach ((int personId, Dictionary<int, List<Vector3>> posesByFrame) in alphaPosesByFrameByPerson)
        {
            if (posesByFrame.TryGetValue(frameCount, out List<Vector3>? value))
            {
                posesByPersonAtFrame.Add(personId, value);
            }
        }

        FrameNumberText.Text = $"{frameCount}:{totalFrameCount}";

        Dispatcher.UIThread.Post(() => { PreviewImage.Source = frame; }, DispatcherPriority.Render);
        Dispatcher.UIThread.RunJobs();

        RedrawPoses();
    }

    void SaveIndicesForFrame()
    {
        // save the lead and follow indices from the last frame
        finalDancerPoses[frameCount][0] = currentLeadIndex > -1 &&
                                          posesByPersonAtFrame.TryGetValue(currentLeadIndex,
                                              out List<Vector3>? lVal)
            ? lVal
            : [];
        finalDancerPoses[frameCount][1] = currentFollowIndex > -1 &&
                                          posesByPersonAtFrame.TryGetValue(currentFollowIndex,
                                              out List<Vector3>? fVal)
            ? fVal
            : [];
        finalDancerPoses[frameCount][2] = mirrorCurrentLeadIndex > -1 &&
                                          posesByPersonAtFrame.TryGetValue(mirrorCurrentLeadIndex,
                                              out List<Vector3>? mirLeadVal)
            ? mirLeadVal
            : [];
        finalDancerPoses[frameCount][3] = mirrorCurrentFollowIndex > -1 &&
                                          posesByPersonAtFrame.TryGetValue(mirrorCurrentFollowIndex,
                                              out List<Vector3>? mirFolVal)
            ? mirFolVal
            : [];

        // save the camera indices from the last frame
        finalIndexCamerasAndPoseAnchor[frameCount].Clear();
        finalIndexMirroredCamerasAndPoseAnchor[frameCount].Clear();
        for (int i = 0; i < numCameras; i++)
        {
            finalIndexCamerasAndPoseAnchor[frameCount].Add(currentSelectedCamerasAndPoseAnchor[i]);
            finalIndexMirroredCamerasAndPoseAnchor[frameCount].Add(mirrorCurrentSelectedCamerasAndPoseAnchor[i]);
        }
    }

    void RedrawPoses()
    {
        Dispatcher.UIThread.Post(() =>
        {
            DrawingImage drawingImage = PreviewDrawer.DrawGeometry(
                posesByPersonAtFrame,
                PreviewImage.Bounds.Size,
                currentLeadIndex,
                currentFollowIndex,
                mirrorCurrentLeadIndex,
                mirrorCurrentFollowIndex,
                currentSelectedCamerasAndPoseAnchor,
                mirrorCurrentSelectedCamerasAndPoseAnchor);
            PoseImage.Source = drawingImage;
        }, DispatcherPriority.Render);

        Dispatcher.UIThread.RunJobs();
    }

    void SetDancer(Vector2 position)
    {
        int closestIndex = -1;
        int jointSelected = -1;
        float closestDistance = float.MaxValue;
        foreach ((int personIndex, List<Vector3> pose) in posesByPersonAtFrame)
        {
            foreach (Vector3 joint in pose)
            {
                if (Vector2.Distance(position, new Vector2(joint.X, joint.Y)) < closestDistance)
                {
                    closestIndex = personIndex;
                    jointSelected = pose.IndexOf(joint);
                    closestDistance = Vector2.Distance(position, new Vector2(joint.X, joint.Y));
                }
            }
        }

        string selectedButton = GetSelectedButton();
        switch (selectedButton)
        {
            case "Lead":
                if (IsMirroredCheckbox.IsChecked == true)
                {
                    mirrorCurrentLeadIndex = mirrorCurrentLeadIndex > -1 && closestIndex == mirrorCurrentLeadIndex
                        ? -1
                        : closestIndex;
                }
                else
                {
                    currentLeadIndex = currentLeadIndex > -1 && closestIndex == currentLeadIndex
                        ? -1 // deselect
                        : closestIndex; // select
                }

                break;
            case "Follow":
                if (IsMirroredCheckbox.IsChecked == true)
                {
                    // deselect follow
                    mirrorCurrentFollowIndex = mirrorCurrentFollowIndex > -1 && closestIndex == mirrorCurrentFollowIndex
                        ? -1
                        : closestIndex;
                }
                else
                {
                    currentFollowIndex = currentFollowIndex > -1 && closestIndex == currentFollowIndex
                        ? -1
                        : closestIndex;
                }

                break;
            default:
                int camNumber = int.Parse(selectedButton[6..]);
                if (IsMirroredCheckbox.IsChecked == true)
                {
                    Tuple<int, bool> camAndHand = mirrorCurrentSelectedCamerasAndPoseAnchor[camNumber];
                    mirrorCurrentSelectedCamerasAndPoseAnchor[camNumber] = camAndHand.Item1 == closestIndex
                        ? new Tuple<int, bool>(-1, false)
                        : new Tuple<int, bool>(
                            closestIndex,
                            HalpeExtension.IsRightSide((Halpe)jointSelected));
                }
                else
                {
                    Tuple<int, bool> camAndHand = currentSelectedCamerasAndPoseAnchor[camNumber];
                    currentSelectedCamerasAndPoseAnchor[camNumber] = camAndHand.Item1 == closestIndex
                        ? new Tuple<int, bool>(-1, false)
                        : new Tuple<int, bool>(
                            closestIndex,
                            HalpeExtension.IsRightSide((Halpe)jointSelected));
                }

                break;
        }

        RedrawPoses();
    }

    void PointerPressedHandler(object sender, PointerPressedEventArgs args)
    {
        PointerPoint point = args.GetCurrentPoint(sender as Control);

        double x = point.Position.X;
        double y = point.Position.Y;

        if (point.Properties.IsLeftButtonPressed)
        {
            SetDancer(new Vector2((float)x, (float)y));
        }
    }

    #endregion

    #region REFERENCE

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

    int FindMaxFrame()
    {
        return alphaPosesByFrameByPerson.Values
            .Aggregate(0,
                (current, posesByFrames) => posesByFrames.Keys.Prepend(current)
                    .Max());
    }

    void SaveTo(string directory, string cameraName)
    {
        string leadSavePath = Path.Combine(directory, $"lead-{cameraName}.json");
        string followSavePath = Path.Combine(directory, $"follow-{cameraName}.json");
        string mirroredLeadSavePath = Path.Combine(directory, $"mirror-lead-{cameraName}.json");
        string mirroredFollowSavePath = Path.Combine(directory, $"mirror-follow-{cameraName}.json");
        string cameraSavePath = Path.Combine(directory, $"camera-{cameraName}.json");
        string mirroredCameraSavePath = Path.Combine(directory, $"mirror-camera-{cameraName}.json");

        File.WriteAllText(leadSavePath,
            JsonConvert.SerializeObject(finalDancerPoses.Select(x => x[0]), Formatting.Indented));
        File.WriteAllText(followSavePath,
            JsonConvert.SerializeObject(finalDancerPoses.Select(x => x[1]), Formatting.Indented));
        File.WriteAllText(mirroredLeadSavePath,
            JsonConvert.SerializeObject(finalDancerPoses.Select(x => x[2]), Formatting.Indented));
        File.WriteAllText(mirroredFollowSavePath,
            JsonConvert.SerializeObject(finalDancerPoses.Select(x => x[3]), Formatting.Indented));

        File.WriteAllText(cameraSavePath,
            JsonConvert.SerializeObject(finalIndexCamerasAndPoseAnchor, Formatting.Indented));
        File.WriteAllText(mirroredCameraSavePath,
            JsonConvert.SerializeObject(finalIndexMirroredCamerasAndPoseAnchor, Formatting.Indented));

        Console.WriteLine($"Saved to: {directory})");
    }

    float PoseError(int personIdx)
    {
        int poseCount = 0;
        List<Vector3> currentPose = [];
        if (alphaPosesByFrameByPerson[personIdx].ContainsKey(frameCount))
        {
            currentPose = alphaPosesByFrameByPerson[personIdx][frameCount];
            poseCount = currentPose.Count;
        }

        List<Vector3> previousPose = [];
        if (alphaPosesByFrameByPerson[personIdx].ContainsKey(frameCount - 1))
        {
            previousPose = alphaPosesByFrameByPerson[personIdx][frameCount - 1];
        }

        float error = 0;
        for (int i = 0; i < poseCount; i++)
        {
            if (currentPose.Count != 0 && previousPose.Count != 0 &&
                currentPose.Count == previousPose.Count)
            {
                error += Vector2.Distance(
                             new Vector2(currentPose[i].X, currentPose[i].Y),
                             new Vector2(previousPose[i].X, previousPose[i].Y))
                         * currentPose[i].Z * previousPose[i].Z; // multiply by confidence
            }
        }

        Console.WriteLine($"pose error: {error}");
        return error;
    }

    bool Continuity()
    {
        if (currentLeadIndex == -1 || currentFollowIndex == -1)
        {
            return false;
        }

        if (!posesByPersonAtFrame.ContainsKey(currentLeadIndex)) return false;
        if (!posesByPersonAtFrame.ContainsKey(currentFollowIndex)) return false;

        if (mirrorCurrentFollowIndex > -1 && !posesByPersonAtFrame.ContainsKey(mirrorCurrentFollowIndex)) return false;
        if (mirrorCurrentLeadIndex > -1 && !posesByPersonAtFrame.ContainsKey(mirrorCurrentLeadIndex)) return false;

        return true;
    }

    string GetVideoPath()
    {
        return VideoFilesDropdown.SelectedItem?.ToString() ?? "";
    }

    string GetAlphaPoseJsonPath()
    {
        string videoPath = GetVideoPath();
        string fileName = Path.GetFileNameWithoutExtension(videoPath);
        return $"{AlphaPoseJsonPath.Text}/{fileName}/alphapose-results.json";
    }

    #endregion

    #region ROOM LAYOUT

    void LayoutCanvas_MouseDown(object sender, PointerPressedEventArgs args)
    {
        // Draw the triangle representing the camera position
        PointerPoint point = args.GetCurrentPoint(sender as Control);

        // Calculate the triangle size based on the focal length
        float focalLength = float.Parse(FocalLengthInput.Text);

        // Create and add the triangle to the canvas
        Polygon triangle = CreateTriangle(point, focalLength, LayoutCanvas.Width, LayoutCanvas.Height);
        LayoutCanvas.Children.Add(triangle);

        cameraPositions.Add(new Vector3((float)point.Position.X, float.Parse(HeightInputText.Text),
            (float)point.Position.Y));
        cameraFocalLengths.Add(focalLength);
    }

    static Polygon CreateTriangle(PointerPoint position, double focalLength, double canvasWidth, double canvasHeight)
    {
        Polygon triangle = new Polygon
        {
            Stroke = Brushes.Black,
            Fill = Brushes.Black,
            StrokeThickness = 2
        };

        // The points of the triangle will be determined based on the position and baseWidth
        const double baseWidth = 30;

        // Calculate the angle to rotate the triangle
        double angle = Math.Atan2(position.Position.Y - canvasHeight / 2, position.Position.X - canvasWidth / 2) -
                       Math.PI / 2;

        // Calculate the rotated points
        Point top = new Point(0, 0);
        Point bottomLeft = new Point(-baseWidth / 2, -focalLength * 100);
        Point bottomRight = new Point(baseWidth / 2, -focalLength * 100);

        triangle.Points.Add(top);
        triangle.Points.Add(RotatePoint(bottomLeft, angle));
        triangle.Points.Add(RotatePoint(bottomRight, angle));

        // Translate the triangle to the position
        Matrix matrix = Matrix.CreateTranslation(position.Position.X, position.Position.Y);
        triangle.RenderTransform = new MatrixTransform(matrix);

        return triangle;
    }

    static Point RotatePoint(Point point, double angle)
    {
        return new Point(
            point.X * Math.Cos(angle) - point.Y * Math.Sin(angle),
            point.X * Math.Sin(angle) + point.Y * Math.Cos(angle)
        );
    }

    #endregion

    #region PERSPECTIVE

    void LoadJsonForPerspectiveButton_Click(object sender, RoutedEventArgs e)
    {
        string directoryPath = DirectoryPathTextBox.Text;

        if (Directory.Exists(directoryPath))
        {
            CanvasContainer.Items.Clear();
            foreach (string file in Directory.GetFiles(directoryPath, "*.json"))
            {
                try
                {
                    // TODO move this out to be able to load multiple files
                    string jsonContent = File.ReadAllText(file);
                    List<List<Vector3>> dancerPoses = JsonConvert.DeserializeObject<List<List<Vector3>>>(jsonContent);
                    Dictionary<int, List<Vector3>> dancersAtFrame = new Dictionary<int, List<Vector3>>();
                    dancersAtFrame.Add(0, dancerPoses[0]);
                    Image image = new Image();
                    Canvas canvas = new Canvas();
                    canvas.Children.Add(image);
                    canvas.Width = 500;
                    canvas.Height = 500;
                    CanvasContainer.Items.Add(canvas);

                    Dispatcher.UIThread.Post(() =>
                    {
                        DrawingImage drawingImage = PreviewDrawer.DrawGeometry(
                            dancersAtFrame,
                            new Size(500, 500),
                            0,
                            -1, // TODO
                            -1,
                            -1,
                            [],
                            []);
                        image.Source = drawingImage;
                    }, DispatcherPriority.Render);

                    Dispatcher.UIThread.RunJobs();
                }
                catch (JsonException jsonEx)
                {
                    // Handle JSON deserialization errors
                    Console.WriteLine($"JSON Error in file {file}: {jsonEx.Message}");
                }
                catch (Exception ex)
                {
                    // Handle other errors (e.g., file read errors)
                    Console.WriteLine($"Error loading file {file}: {ex.Message}");
                }
            }
        }
        else
        {
            // Handle the case where the directory does not exist
            Console.WriteLine("Directory does not exist.");
        }
    }

    void Canvas_PointerPressed(object sender, PointerPressedEventArgs e)
    {
        // Mark this canvas as selected
    }

    // Add event handlers for perspective manipulation buttons
    void YawLeftButton_Click(object sender, RoutedEventArgs e)
    {
        /* ... */
    }

    void YawRightButton_Click(object sender, RoutedEventArgs e)
    {
        /* ... */
    }

    void PitchUpButton_Click(object sender, RoutedEventArgs e)
    {
        /* ... */
    }

    void PitchDownButton_Click(object sender, RoutedEventArgs e)
    {
        /* ... */
    }

    void ZoomInButton_Click(object sender, RoutedEventArgs e)
    {
        /* ... */
    }

    void ZoomOutButton_Click(object sender, RoutedEventArgs e)
    {
        /* ... */
    }

    void RollLeftButton_Click(object sender, RoutedEventArgs e)
    {
        /* ... */
    }

    void RollRightButton_Click(object sender, RoutedEventArgs e)
    {
        /* ... */
    }

    void TranslateLeftButton_Click(object sender, RoutedEventArgs e)
    {
        /* ... */
    }

    void TranslateRightButton_Click(object sender, RoutedEventArgs e)
    {
        /* ... */
    }

    void TranslateUpButton_Click(object sender, RoutedEventArgs e)
    {
        /* ... */
    }

    void TranslateDownButton_Click(object sender, RoutedEventArgs e)
    {
        /* ... */
    }

    void TranslateForwardButton_Click(object sender, RoutedEventArgs e)
    {
        /* ... */
    }

    void TranslateBackwardButton_Click(object sender, RoutedEventArgs e)
    {
        /* ... */
    }

    void SolveButton_Click(object sender, RoutedEventArgs e)
    {
        // Implement the solve logic
    }

    void Save3D_Click(object sender, RoutedEventArgs e)
    {
        // Implement the save logic
    }

    #endregion
}