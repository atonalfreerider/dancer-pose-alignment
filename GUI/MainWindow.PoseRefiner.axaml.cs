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
using Path = System.IO.Path;
using Window = Avalonia.Controls.Window;

namespace GUI;

public partial class MainWindow : Window
{
    // initializeation
    VideoCapture videoCapture;
    Dictionary<int, Dictionary<int, List<Vector3>>> alphaPosesByFrameByPerson;
    readonly List<Image> alignmentImages = [];

    // indices for tracked figures
    int currentLeadIndex = -1;
    int mirrorCurrentLeadIndex = -1;
    int currentFollowIndex = -1;
    int mirrorCurrentFollowIndex = -1;

    // frame numbers
    int numCameras = 0;
    int videoFrameIndex = 0;
    int totalVideoFrameCount = 0;

    // current state of frame
    Dictionary<int, List<Vector3>> posesByPersonAtFrame = new();
    readonly List<Tuple<int, bool>> currentSelectedCamerasAndPoseAnchor = []; // right or left hand
    readonly List<Tuple<int, bool>> mirrorCurrentSelectedCamerasAndPoseAnchor = []; // right or left hand

    // states to serialize
    // 0 - lead, 1 - follow, 2 - mirror lead, 3 - mirror follow. they are updated at every frame change
    readonly List<List<List<Vector3>>> finalDancerPoses = [];
    readonly List<List<Tuple<int, bool>>> finalCamerasIndicesAndHandByFrame = [];
    readonly List<List<Tuple<int, bool>>> mirrorFinalCamerasIndicesAndHandByFrame = [];

    public MainWindow()
    {
        InitializeComponent();
        VideoFilesDropdown.SelectionChanged += delegate
        {
            // reset the state
            videoFrameIndex = 0;
            currentLeadIndex = -1;
            mirrorCurrentLeadIndex = -1;
            currentFollowIndex = -1;
            mirrorCurrentFollowIndex = -1;
            posesByPersonAtFrame = new Dictionary<int, List<Vector3>>();
            finalDancerPoses.Clear();
            finalCamerasIndicesAndHandByFrame.Clear();
            mirrorFinalCamerasIndicesAndHandByFrame.Clear();
            ClearCameraMarkers();

            alphaPosesByFrameByPerson = AlphaPose.PosesByFrameByPerson(GetAlphaPoseJsonPath());
            totalVideoFrameCount = FindMaxFrame();
            // populate final index lists
            for (int i = 0; i < totalVideoFrameCount; i++)
            {
                List<List<Vector3>> frameDancerPoses = [];
                for (int j = 0; j < 4; j++)
                {
                    frameDancerPoses.Add([]); // 0 - lead, 1 - follow, 2 - mirror lead, 3 - mirror follow
                }

                finalDancerPoses.Add(frameDancerPoses);
                finalCamerasIndicesAndHandByFrame.Add([]);
                mirrorFinalCamerasIndicesAndHandByFrame.Add([]);
            }

            videoCapture = new VideoCapture(GetVideoPath());
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
        if (videoFrameIndex <= 0) return;
        SaveIndicesForFrame();
        videoFrameIndex--;
        RenderFrame();
    }

    void NextFrameButton_Click(object sender, RoutedEventArgs e)
    {
        if (videoFrameIndex >= totalVideoFrameCount) return;

        SaveIndicesForFrame();
        videoFrameIndex++;
        RenderFrame();
    }

    void RunUntilNext_Click(object sender, RoutedEventArgs e)
    {
        while (videoFrameIndex < totalVideoFrameCount && Continuity())
        {
            SaveIndicesForFrame();
            videoFrameIndex++;
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
        videoCapture.Set(VideoCaptureProperties.PosFrames, videoFrameIndex);

        OutputArray outputArray = new Mat();
        videoCapture.Read(outputArray);

        Mat frameMat = outputArray.GetMat();
        VideoCanvas.Width = frameMat.Width;
        VideoCanvas.Height = frameMat.Height;

        RenderFrame();
    }

    void RenderFrame()
    {
        OutputArray outputArray = new Mat();
        videoCapture.Set(VideoCaptureProperties.PosFrames, videoFrameIndex);
        videoCapture.Read(outputArray);

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
            if (posesByFrame.TryGetValue(videoFrameIndex, out List<Vector3>? value))
            {
                posesByPersonAtFrame.Add(personId, value);
            }
        }

        RefinerFrameNumberText.Text = $"{videoFrameIndex}:{totalVideoFrameCount}";

        Dispatcher.UIThread.Post(() => { PreviewImage.Source = frame; }, DispatcherPriority.Render);
        Dispatcher.UIThread.RunJobs();

        RedrawPoses();
    }

    void SaveIndicesForFrame()
    {
        // save the lead and follow indices from the last frame
        finalDancerPoses[videoFrameIndex][0] = currentLeadIndex > -1 &&
                                          posesByPersonAtFrame.TryGetValue(currentLeadIndex,
                                              out List<Vector3>? lVal)
            ? lVal
            : [];
        finalDancerPoses[videoFrameIndex][1] = currentFollowIndex > -1 &&
                                          posesByPersonAtFrame.TryGetValue(currentFollowIndex,
                                              out List<Vector3>? fVal)
            ? fVal
            : [];
        finalDancerPoses[videoFrameIndex][2] = mirrorCurrentLeadIndex > -1 &&
                                          posesByPersonAtFrame.TryGetValue(mirrorCurrentLeadIndex,
                                              out List<Vector3>? mirLeadVal)
            ? mirLeadVal
            : [];
        finalDancerPoses[videoFrameIndex][3] = mirrorCurrentFollowIndex > -1 &&
                                          posesByPersonAtFrame.TryGetValue(mirrorCurrentFollowIndex,
                                              out List<Vector3>? mirFolVal)
            ? mirFolVal
            : [];

        // save the camera indices from the last frame
        finalCamerasIndicesAndHandByFrame[videoFrameIndex].Clear();
        mirrorFinalCamerasIndicesAndHandByFrame[videoFrameIndex].Clear();
        for (int i = 0; i < numCameras; i++)
        {
            finalCamerasIndicesAndHandByFrame[videoFrameIndex].Add(currentSelectedCamerasAndPoseAnchor[i]);
            mirrorFinalCamerasIndicesAndHandByFrame[videoFrameIndex].Add(mirrorCurrentSelectedCamerasAndPoseAnchor[i]);
        }
    }

    void RedrawPoses()
    {
        Dispatcher.UIThread.Post(() =>
        {
            DrawingImage drawingImage = new DrawingImage();
            DrawingGroup drawingGroup = PreviewDrawer.DrawGeometry(
                posesByPersonAtFrame,
                PreviewImage.Bounds.Size,
                currentLeadIndex,
                currentFollowIndex,
                mirrorCurrentLeadIndex,
                mirrorCurrentFollowIndex,
                currentSelectedCamerasAndPoseAnchor,
                mirrorCurrentSelectedCamerasAndPoseAnchor);
            drawingImage.Drawing = drawingGroup;
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

        List<List<Vector3>> otherCameraPositionsByFrame = ExtractCameraHandPositions(finalCamerasIndicesAndHandByFrame);
        List<List<Vector3>> mirroredOtherCameraPositionsByFrame =
            ExtractCameraHandPositions(mirrorFinalCamerasIndicesAndHandByFrame);

        File.WriteAllText(cameraSavePath,
            JsonConvert.SerializeObject(otherCameraPositionsByFrame, Formatting.Indented));
        File.WriteAllText(mirroredCameraSavePath,
            JsonConvert.SerializeObject(mirroredOtherCameraPositionsByFrame, Formatting.Indented));

        Console.WriteLine($"Saved to: {directory})");
    }

    List<List<Vector3>> ExtractCameraHandPositions(IReadOnlyList<List<Tuple<int, bool>>> pass)
    {
        List<List<Vector3>> otherCameraPositionsByFrame = [];
        for (int i = 0; i < totalVideoFrameCount; i++)
        {
            List<Tuple<int, bool>> currentCamerasAndPoseAnchor = pass[i];
            List<Vector3> otherCameraPositions = [];
            for (int j = 0; j < numCameras; j++)
            {
                Tuple<int, bool> camAndHand = currentCamerasAndPoseAnchor[j];
                if (camAndHand.Item1 == -1 || !alphaPosesByFrameByPerson[camAndHand.Item1].ContainsKey(i))
                {
                    otherCameraPositions.Add(new Vector3(-1, -1, 0));
                }
                else
                {
                    List<Vector3> cameraHolderPoses =
                        alphaPosesByFrameByPerson[camAndHand.Item1][i];
                    if (camAndHand.Item2)
                    {
                        // right hand
                        otherCameraPositions.Add(new Vector3(
                            cameraHolderPoses[(int)Halpe.RWrist].X,
                            cameraHolderPoses[(int)Halpe.RWrist].Y,
                            cameraHolderPoses[(int)Halpe.RWrist].Z));
                    }
                    else
                    {
                        otherCameraPositions.Add(new Vector3(
                            cameraHolderPoses[(int)Halpe.LWrist].X,
                            cameraHolderPoses[(int)Halpe.LWrist].Y,
                            cameraHolderPoses[(int)Halpe.LWrist].Z));
                    }
                }
            }

            otherCameraPositionsByFrame.Add(otherCameraPositions);
        }

        return otherCameraPositionsByFrame;
    }

    float PoseError(int personIdx)
    {
        int poseCount = 0;
        List<Vector3> currentPose = [];
        if (alphaPosesByFrameByPerson[personIdx].ContainsKey(videoFrameIndex))
        {
            currentPose = alphaPosesByFrameByPerson[personIdx][videoFrameIndex];
            poseCount = currentPose.Count;
        }

        List<Vector3> previousPose = [];
        if (alphaPosesByFrameByPerson[personIdx].ContainsKey(videoFrameIndex - 1))
        {
            previousPose = alphaPosesByFrameByPerson[personIdx][videoFrameIndex - 1];
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
}