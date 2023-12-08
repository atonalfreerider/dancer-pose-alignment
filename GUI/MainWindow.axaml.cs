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
using Image = Avalonia.Controls.Image;
using Window = Avalonia.Controls.Window;

namespace GUI;

public partial class MainWindow : Window
{
    bool hasVideoBeenInitialized = false;
    bool hasPoseBeenInitialized = false;
    FrameSource frameSource;
    Dictionary<int, Dictionary<int, List<Vector3>>> PosesByFrameByPerson;
    int currentLeadIndex = -1;
    int mirrorCurrentLeadIndex = -1;
    int currentFollowIndex = -1;
    int mirrorCurrentFollowIndex = -1;
    int frameCount = 0;
    Dictionary<int, List<Vector3>> posesByPersonAtFrame = new();
    int totalFrameCount = 0;
    int stepBackFrame = 0;

    readonly List<Tuple<int, int>> currentSelectedCamerasAndPoseAnchor = [];
    readonly List<Tuple<int, int>> mirrorCurrentSelectedCamerasAndPoseAnchor = [];

    readonly List<Tuple<int, int>> finalIndexListLeadAndFollow = [];
    readonly List<Tuple<int, int>> finalIndexListMirroredLeadAndFollow = [];
    readonly List<List<Tuple<int, int>>> finalIndexCamerasAndPoseAnchor = [];
    readonly List<List<Tuple<int, int>>> finalIndexMirroredCamerasAndPoseAnchor = [];

    readonly List<Bitmap> lastTenFrames = [];

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

        if (Directory.Exists(videoDirectory))
        {
            List<string> videoFiles = Directory.EnumerateFiles(videoDirectory, "*.*", SearchOption.TopDirectoryOnly)
                .Where(file => file.EndsWith(".mp4") || file.EndsWith(".avi") || file.EndsWith(".mkv")).ToList();
            VideoFilesDropdown.ItemsSource = videoFiles;

            for (int i = 0; i < videoFiles.Count; i++)
            {
                RadioButton radioButton = new RadioButton
                {
                    GroupName = "Role",
                    Content = $"Camera{i}",
                    Margin = new Thickness(5)
                };

                DynamicRadioButtonsPanel.Children.Add(radioButton);

                currentSelectedCamerasAndPoseAnchor.Add(new Tuple<int, int>(-1, -1));
                mirrorCurrentSelectedCamerasAndPoseAnchor.Add(new Tuple<int, int>(-1, -1));
            }
        }
    }

    void ClearDancers_Click(object sender, RoutedEventArgs e)
    {
        currentLeadIndex = -1;
        currentFollowIndex = -1;
        RedrawPoses();
    }

    void BackButton_Click(object sender, RoutedEventArgs e)
    {
        if (frameCount > 0 && stepBackFrame < 10)
        {
            stepBackFrame++;
            RenderFrame(lastTenFrames[stepBackFrame], GetAlphaPoseJsonPath(), frameCount - stepBackFrame);
        }
    }

    void NextFrameButton_Click(object sender, RoutedEventArgs e)
    {
        if (stepBackFrame > 0)
        {
            stepBackFrame--;
            RenderFrame(lastTenFrames[stepBackFrame], GetAlphaPoseJsonPath(), frameCount - stepBackFrame);
        }
        else
        {
            RenderFrame(GetVideoPath(), GetAlphaPoseJsonPath());
        }
    }

    void RunUntilNext_Click(object sender, RoutedEventArgs e)
    {
        while (frameCount < totalFrameCount &&
               posesByPersonAtFrame.ContainsKey(currentLeadIndex) &&
               posesByPersonAtFrame.ContainsKey(currentFollowIndex))
        {
            RenderFrame(GetVideoPath(), GetAlphaPoseJsonPath());
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

    void RenderFrame(string videoPath, string alphaPoseJsonPath)
    {
        if (frameCount > 0)
        {
            // save the lead and follow indices from the last frame
            finalIndexListLeadAndFollow.Add(new Tuple<int, int>(currentLeadIndex, currentFollowIndex));
        }

        if (!hasVideoBeenInitialized)
        {
            frameSource = Cv2.CreateFrameSource_Video(videoPath);
        }

        OutputArray outputArray = new Mat();
        frameSource.NextFrame(outputArray);

        Mat frameMat = outputArray.GetMat();

        if (!hasVideoBeenInitialized)
        {
            VideoCanvas.Width = frameMat.Width;
            VideoCanvas.Height = frameMat.Height;
            hasVideoBeenInitialized = true;
        }

        Bitmap frame;
        try
        {
            frame = Bitmap.DecodeToWidth(frameMat.ToMemoryStream(), frameMat.Width);
        }
        catch (OpenCVException e)
        {
            // end of video
            Console.WriteLine(e.Message);
            frameCount++;
            return;
        }

        InsertToFrontOfQueue(frame);

        RenderFrame(frame, alphaPoseJsonPath, frameCount);
        frameCount++;
    }

    void RenderFrame(Bitmap frame, string alphaPoseJsonPath, int frameNumber)
    {
        if (!hasPoseBeenInitialized)
        {
            PosesByFrameByPerson = AlphaPose.PosesByFrameByPerson(alphaPoseJsonPath);
            totalFrameCount = FindMaxFrame();
            hasPoseBeenInitialized = true;
        }

        posesByPersonAtFrame = new Dictionary<int, List<Vector3>>();
        foreach ((int personId, Dictionary<int, List<Vector3>> posesByFrame) in PosesByFrameByPerson)
        {
            if (posesByFrame.TryGetValue(frameNumber, out List<Vector3>? value))
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
                    mirrorCurrentSelectedCamerasAndPoseAnchor[camNumber] = new Tuple<int, int>(closestIndex, jointSelected);
                }
                else
                {
                    currentSelectedCamerasAndPoseAnchor[camNumber] = new Tuple<int, int>(closestIndex, jointSelected);
                }
                break;
        }

        RedrawPoses();
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

    void InsertToFrontOfQueue(Bitmap frame)
    {
        lastTenFrames.Insert(0, frame);
        if (lastTenFrames.Count > 10)
        {
            lastTenFrames.RemoveAt(lastTenFrames.Count - 1);
        }
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
}