using System.Numerics;
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
    readonly Image previewImage;
    readonly Image poseImage;
    readonly TextBox frameNumberText;
    readonly Canvas canvas;

    bool hasVideoBeenInitialized = false;
    bool hasPoseBeenInitialized = false;
    FrameSource frameSource;
    Dictionary<int, Dictionary<int, List<Vector3>>> PosesByFrameByPerson;
    int currentLeadIndex = -1;
    int currentFollowIndex = -1;
    int frameCount = 0;
    Dictionary<int, List<Vector3>> posesByPersonAtFrame = new();
    int totalFrameCount = 0;
    int stepBackFrame = 0;

    readonly List<Tuple<int, int>> finalIndexListLeadAndFollow = [];

    readonly List<Bitmap> lastTenFrames = [];

    public MainWindow()
    {
        InitializeComponent();

        frameNumberText = this.Find<TextBox>("FrameNumberText")!;

        canvas = this.Find<Canvas>("Canvas")!;

        previewImage = this.Find<Image>("PreviewImage")!;
        poseImage = this.Find<Image>("PoseImage")!;
    }
    
    void LoadVideosButton_Click(object sender, RoutedEventArgs e)
    {
        string? videoDirectory = VideoInputPath.Text;
        if (string.IsNullOrEmpty(videoDirectory)) return;
        if(!videoDirectory.EndsWith('/') && !videoDirectory.EndsWith('\\'))
        {
            videoDirectory += '/';
        }
        if (Directory.Exists(videoDirectory))
        {
            IEnumerable<string> videoFiles = Directory.EnumerateFiles(videoDirectory, "*.*", SearchOption.TopDirectoryOnly)
                .Where(file => file.EndsWith(".mp4") || file.EndsWith(".avi") || file.EndsWith(".mkv"));
            VideoFilesDropdown.ItemsSource = videoFiles;
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
               posesByPersonAtFrame.ContainsKey(currentFollowIndex) &&
               PoseError() < 200) // arbitrary value from trial and error when figures are about 1/4 of the screen
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
            canvas.Width = frameMat.Width;
            canvas.Height = frameMat.Height;
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

        frameNumberText.Text = $"{frameCount}:{totalFrameCount}";
        SetPreview(frame);
        RedrawPoses();
    }

    void SetPreview(IImage frame)
    {
        Dispatcher.UIThread.Post(() => { previewImage.Source = frame; }, DispatcherPriority.Render);

        Dispatcher.UIThread.RunJobs();
    }

    void RedrawPoses()
    {
        Dispatcher.UIThread.Post(() =>
        {
            DrawingImage drawingImage = PreviewDrawer.DrawGeometry(
                posesByPersonAtFrame,
                previewImage.Bounds.Size,
                currentLeadIndex,
                currentFollowIndex);
            poseImage.Source = drawingImage;
        }, DispatcherPriority.Render);

        Dispatcher.UIThread.RunJobs();
    }

    void SetDancer(Vector2 position)
    {
        int closestIndex = -1;
        float closestDistance = float.MaxValue;
        foreach ((int personIndex, List<Vector3> pose) in posesByPersonAtFrame)
        {
            foreach (Vector3 joint in pose)
            {
                if (Vector2.Distance(position, new Vector2(joint.X, joint.Y)) < closestDistance)
                {
                    closestIndex = personIndex;
                    closestDistance = Vector2.Distance(position, new Vector2(joint.X, joint.Y));
                }
            }
        }

        if (currentLeadIndex > -1 && closestIndex == currentLeadIndex)
        {
            // deselect lead
            currentLeadIndex = -1;
        }
        else if (currentFollowIndex > -1 && closestIndex == currentFollowIndex)
        {
            // deselect follow
            currentFollowIndex = -1;
        }
        else if (currentLeadIndex == -1)
        {
            // set lead
            currentLeadIndex = closestIndex;
        }
        else if (currentFollowIndex == -1)
        {
            // set follow
            currentFollowIndex = closestIndex;
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

    float PoseError()
    {
        int poseCount = 0;
        List<Vector3> currentLeadPoses = [];
        if (PosesByFrameByPerson[currentLeadIndex].ContainsKey(frameCount))
        {
            currentLeadPoses = PosesByFrameByPerson[currentLeadIndex][frameCount];
            poseCount = currentLeadPoses.Count;
        }

        List<Vector3> currentFollowPoses = [];
        if (PosesByFrameByPerson[currentFollowIndex].ContainsKey(frameCount))
        {
            currentFollowPoses = PosesByFrameByPerson[currentFollowIndex][frameCount];
            poseCount = currentFollowPoses.Count;
        }

        List<Vector3> previousLeadPoses = [];
        if (PosesByFrameByPerson[currentLeadIndex].ContainsKey(frameCount - 1))
        {
            previousLeadPoses = PosesByFrameByPerson[currentLeadIndex][frameCount - 1];
        }

        List<Vector3> previousFollowPoses = [];
        if (PosesByFrameByPerson[currentFollowIndex].ContainsKey(frameCount - 1))
        {
            previousFollowPoses = PosesByFrameByPerson[currentFollowIndex][frameCount - 1];
        }

        float error = 0;
        for (int i = 0; i < poseCount; i++)
        {
            if (currentLeadPoses.Count != 0 && previousLeadPoses.Count != 0 &&
                currentLeadPoses.Count == previousLeadPoses.Count)
            {
                error += Vector2.Distance(
                             new Vector2(currentLeadPoses[i].X, currentLeadPoses[i].Y),
                             new Vector2(previousLeadPoses[i].X, previousLeadPoses[i].Y))
                         * currentLeadPoses[i].Z * previousLeadPoses[i].Z; // multiply by confidence
            }

            if (currentFollowPoses.Count != 0 && previousFollowPoses.Count != 0 &&
                currentFollowPoses.Count == previousFollowPoses.Count)
            {
                error += Vector2.Distance(
                             new Vector2(currentFollowPoses[i].X, currentFollowPoses[i].Y),
                             new Vector2(previousFollowPoses[i].X, previousFollowPoses[i].Y))
                         * currentFollowPoses[i].Z * previousFollowPoses[i].Z; // multiply by confidence
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