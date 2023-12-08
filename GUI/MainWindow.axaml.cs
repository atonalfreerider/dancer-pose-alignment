using System.Numerics;
using Avalonia;
using Avalonia.Controls;
using Avalonia.Input;
using Avalonia.Interactivity;
using Avalonia.Markup.Xaml;
using Avalonia.Media;
using Avalonia.Media.Imaging;
using Avalonia.Threading;
using dancer_pose_alignment;
using Newtonsoft.Json;
using OpenCvSharp;
using Window = Avalonia.Controls.Window;

namespace GUI;

public partial class MainWindow : Window
{
    // initializeation
    VideoCapture frameSource;
    Dictionary<int, Dictionary<int, List<Vector3>>> PosesByFrameByPerson;

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
    readonly List<Tuple<int, int>> finalIndexListLeadAndFollow = [];
    readonly List<Tuple<int, int>> finalIndexListMirroredLeadAndFollow = [];
    readonly List<List<Tuple<int, bool>>> finalIndexCamerasAndPoseAnchor = [];
    readonly List<List<Tuple<int, bool>>> finalIndexMirroredCamerasAndPoseAnchor = [];

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
            finalIndexListLeadAndFollow.Clear();
            finalIndexListMirroredLeadAndFollow.Clear();
            finalIndexCamerasAndPoseAnchor.Clear();
            finalIndexMirroredCamerasAndPoseAnchor.Clear();
            ResetCamMarkers();

            PosesByFrameByPerson = AlphaPose.PosesByFrameByPerson(GetAlphaPoseJsonPath());
            totalFrameCount = FindMaxFrame();

            frameSource = new VideoCapture(GetVideoPath());
            RenderZero();
        };
    }

    #region BUTTON ACTIONS

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

                currentSelectedCamerasAndPoseAnchor.Add(new Tuple<int, bool>(-1, false));
                mirrorCurrentSelectedCamerasAndPoseAnchor.Add(new Tuple<int, bool>(-1, false));
            }
        }
    }

    void ClearDancers_Click(object sender, RoutedEventArgs e)
    {
        currentLeadIndex = -1;
        currentFollowIndex = -1;
        mirrorCurrentFollowIndex = -1;
        mirrorCurrentLeadIndex = -1;

        ResetCamMarkers();
        RedrawPoses();
    }

    void ResetCamMarkers()
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

        frameCount--;
        RenderFrame();
    }

    void NextFrameButton_Click(object sender, RoutedEventArgs e)
    {
        if (frameCount >= totalFrameCount) return;

        frameCount++;
        RenderFrame();
    }

    void RunUntilNext_Click(object sender, RoutedEventArgs e)
    {
        while (frameCount < totalFrameCount &&
               posesByPersonAtFrame.ContainsKey(currentLeadIndex) &&
               posesByPersonAtFrame.ContainsKey(currentFollowIndex))
        {
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
            cameraName = Path.GetFileNameWithoutExtension(VideoInputPath.Text);
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
        if (frameCount > 0)
        {
            // save the lead and follow indices from the last frame
            finalIndexListLeadAndFollow.Add(new Tuple<int, int>(currentLeadIndex, currentFollowIndex));
        }

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
        foreach ((int personId, Dictionary<int, List<Vector3>> posesByFrame) in PosesByFrameByPerson)
        {
            if (posesByFrame.TryGetValue(frameCount, out List<Vector3>? value))
            {
                posesByPersonAtFrame.Add(personId, value);
            }
        }

        FrameNumberText.Text = $"{frameCount}:{totalFrameCount}";
        SetPreview(frame);
        RedrawPoses();
    }

    void SetPreview(IImage frame)
    {
        Dispatcher.UIThread.Post(() => { PreviewImage.Source = frame; }, DispatcherPriority.Render);

        Dispatcher.UIThread.RunJobs();
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
                    if (mirrorCurrentLeadIndex > -1 && closestIndex == mirrorCurrentLeadIndex)
                    {
                        // deselect lead
                        mirrorCurrentLeadIndex = -1;
                    }
                    else
                    {
                        // set lead
                        mirrorCurrentLeadIndex = closestIndex;
                    }
                }
                else
                {
                    if (currentLeadIndex > -1 && closestIndex == currentLeadIndex)
                    {
                        // deselect lead
                        currentLeadIndex = -1;
                    }
                    else
                    {
                        // set lead
                        currentLeadIndex = closestIndex;
                    }
                }

                break;
            case "Follow":
                if (IsMirroredCheckbox.IsChecked == true)
                {
                    if (mirrorCurrentFollowIndex > -1 && closestIndex == mirrorCurrentFollowIndex)
                    {
                        // deselect follow
                        mirrorCurrentFollowIndex = -1;
                    }
                    else
                    {
                        // set follow
                        mirrorCurrentFollowIndex = closestIndex;
                    }
                }
                else
                {
                    if (currentFollowIndex > -1 && closestIndex == currentFollowIndex)
                    {
                        // deselect follow
                        currentFollowIndex = -1;
                    }
                    else
                    {
                        // set follow
                        currentFollowIndex = closestIndex;
                    }
                }

                break;
            default:
                int camNumber = int.Parse(selectedButton[6..]);
                if (IsMirroredCheckbox.IsChecked == true)
                {
                    Tuple<int, bool> camAndHand = mirrorCurrentSelectedCamerasAndPoseAnchor[camNumber];
                    if (camAndHand.Item1 == closestIndex)
                    {
                        mirrorCurrentSelectedCamerasAndPoseAnchor[camNumber] = new Tuple<int, bool>(-1, false);
                    }
                    else
                    {
                        mirrorCurrentSelectedCamerasAndPoseAnchor[camNumber] = new Tuple<int, bool>(
                            closestIndex,
                            HalpeExtension.IsRightSide((Halpe) jointSelected));
                    }
                }
                else
                {
                    Tuple<int, bool> camAndHand = currentSelectedCamerasAndPoseAnchor[camNumber];
                    if (camAndHand.Item1 == closestIndex)
                    {
                        currentSelectedCamerasAndPoseAnchor[camNumber] = new Tuple<int, bool>(-1, false);
                    }
                    else
                    {
                        currentSelectedCamerasAndPoseAnchor[camNumber] = new Tuple<int, bool>(
                            closestIndex,
                            HalpeExtension.IsRightSide((Halpe) jointSelected));
                    }
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
        return PosesByFrameByPerson.Values
            .Aggregate(0,
                (current, posesByFrames) => posesByFrames.Keys.Prepend(current)
                    .Max());
    }

    void SaveTo(string directory, string cameraName)
    {
        string leadSavePath = Path.Combine(directory, $"lead-{cameraName}.json");
        string followSavePath = Path.Combine(directory, $"follow-{cameraName}.json");

        List<List<Vector3>> leadPoses = [];
        List<List<Vector3>> followPoses = [];
        int count = 0;
        foreach (Tuple<int, int> leadAndFollow in finalIndexListLeadAndFollow)
        {
            if (leadAndFollow.Item1 > -1 &&
                PosesByFrameByPerson[leadAndFollow.Item1].TryGetValue(count, out List<Vector3>? leadVal))
            {
                leadPoses.Add(leadVal);
            }
            else
            {
                // add empty if pose is missing
                leadPoses.Add([]);
            }

            if (leadAndFollow.Item2 > -1 &&
                PosesByFrameByPerson[leadAndFollow.Item2].TryGetValue(count, out List<Vector3>? followVal))
            {
                followPoses.Add(followVal);
            }
            else
            {
                // add empty if pose is missing
                followPoses.Add([]);
            }

            count++;
        }

        File.WriteAllText(leadSavePath, JsonConvert.SerializeObject(leadPoses, Formatting.Indented));
        File.WriteAllText(followSavePath, JsonConvert.SerializeObject(followPoses, Formatting.Indented));

        Console.WriteLine($"Saved to: {leadSavePath})");
        Console.WriteLine($"Saved to: {followSavePath})");
    }

    float PoseError(int personIdx)
    {
        int poseCount = 0;
        List<Vector3> currentPose = [];
        if (PosesByFrameByPerson[personIdx].ContainsKey(frameCount))
        {
            currentPose = PosesByFrameByPerson[personIdx][frameCount];
            poseCount = currentPose.Count;
        }

        List<Vector3> previousPose = [];
        if (PosesByFrameByPerson[personIdx].ContainsKey(frameCount - 1))
        {
            previousPose = PosesByFrameByPerson[personIdx][frameCount - 1];
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
}