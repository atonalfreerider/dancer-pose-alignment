using System.Numerics;
using Avalonia.Controls;
using Avalonia.Input;
using Avalonia.Markup.Xaml;
using Avalonia.Media;
using Avalonia.Media.Imaging;
using Avalonia.Threading;
using dancer_pose_alignment;
using OpenCvSharp;
using Image = Avalonia.Controls.Image;
using Window = Avalonia.Controls.Window;

namespace GUI;

public partial class MainWindow : Window
{
    readonly Image previewImage;
    readonly Image poseImage;
    readonly TextBox frameNumberText;

    bool hasBeenInitialized = false;
    FrameSource frameSource;
    Dictionary<int, Dictionary<int, List<Vector3>>> PosesByFrameByPerson;
    int currentLeadIndex = -1;
    int currentFollowIndex = -1;
    int frameCount = 0;
    Dictionary<int, List<Vector3>> posesByPersonAtFrame = new();
    int totalFrameCount = 0;

    public MainWindow()
    {
        AvaloniaXamlLoader.Load(this);

        TextBox videoInputPath = this.Find<TextBox>("VideoInputPath")!;
        TextBox alphaPoseJsonPath = this.Find<TextBox>("AlphaPoseJsonPath")!;
        frameNumberText = this.Find<TextBox>("FrameNumberText")!;

        Button nextFrameButton = this.Find<Button>("NextFrameButton")!;

        nextFrameButton.Click += delegate { RenderFrame(videoInputPath.Text, alphaPoseJsonPath.Text); };

        Button runUntilNextButton = this.Find<Button>("RunUntilNext")!;

        runUntilNextButton.Click += delegate
        {
            while (posesByPersonAtFrame.ContainsKey(currentLeadIndex) &&
                   posesByPersonAtFrame.ContainsKey(currentFollowIndex))
            {
                RenderFrame(videoInputPath.Text, alphaPoseJsonPath.Text);
            }
        };

        previewImage = this.Find<Image>("PreviewImage")!;
        poseImage = this.Find<Image>("PoseImage")!;
    }

    void RenderFrame(string videoPath, string alphaPoseJsonPath)
    {
        if (!hasBeenInitialized)
        {
            frameSource = Cv2.CreateFrameSource_Video(videoPath);
            PosesByFrameByPerson = AlphaPose.PosesByFrameByPerson(alphaPoseJsonPath);
            totalFrameCount = FindMaxFrame();
            hasBeenInitialized = true;
        }

        OutputArray outputArray = new Mat();
        frameSource.NextFrame(outputArray);

        Mat frameMat = outputArray.GetMat();
        Bitmap frame = Bitmap.DecodeToWidth(frameMat.ToMemoryStream(), frameMat.Width);

        posesByPersonAtFrame = new Dictionary<int, List<Vector3>>();
        foreach ((int personId, Dictionary<int, List<Vector3>> posesByFrame) in PosesByFrameByPerson)
        {
            if (posesByFrame.ContainsKey(frameCount))
            {
                posesByPersonAtFrame.Add(personId, posesByFrame[frameCount]);
            }
        }

        frameNumberText.Text = $"{frameCount}:{totalFrameCount}";
        SetPreview(frame);
        RedrawPoses();

        frameCount++;
    }

    void SetPreview(IImage frame)
    {
        Dispatcher.UIThread.Post(() =>
        {
            previewImage.Source = frame;
        }, DispatcherPriority.Render);

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
            return;
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
}