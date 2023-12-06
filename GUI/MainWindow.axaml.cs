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

    bool hasBeenInitialized = false;
    FrameSource frameSource;
    Dictionary<int, Dictionary<int, List<Vector3>>> PosesByFrameByPerson;
    int currentLeadIndex = -1;
    int currentFollowIndex = -1;
    int frameCount = 0;
    Dictionary<int, List<Vector3>> posesByPersonAtFrame = new();

    delegate void UpdatePreview(Bitmap frame);

    UpdatePreview updatePreview;

    public MainWindow()
    {
        AvaloniaXamlLoader.Load(this);

        TextBox videoInputPath = this.Find<TextBox>("VideoInputPath");
        TextBox alphaPoseJsonPath = this.Find<TextBox>("AlphaPoseJsonPath");

        Button nextFrameButton = this.Find<Button>("NextFrameButton");

        nextFrameButton.Click += delegate { RenderFrame(videoInputPath.Text, alphaPoseJsonPath.Text); };

        previewImage = this.Find<Image>("PreviewImage");
        poseImage = this.Find<Image>("PoseImage");
    }

    public void SetupPreview()
    {
        updatePreview = SetPreview;
    }

    void RenderFrame(string videoPath, string alphaPoseJsonPath)
    {
        if (!hasBeenInitialized)
        {
            frameSource = Cv2.CreateFrameSource_Video(videoPath);
            PosesByFrameByPerson = AlphaPose.PosesByFrameByPerson(alphaPoseJsonPath);
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

        updatePreview.Invoke(frame);

        frameCount++;
    }

    void SetPreview(Bitmap frame)
    {
        Dispatcher.UIThread.Post(() =>
        {
            previewImage.Source = frame;
            RedrawPoses();
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
            foreach(Vector3 joint in pose)
            {
                if(Vector2.Distance(position, new Vector2(joint.X, joint.Y)) < closestDistance)
                {
                    closestIndex = personIndex;
                    closestDistance = Vector2.Distance(position, new Vector2(joint.X, joint.Y));
                }
            }
        }

        if (currentLeadIndex == -1)
        {
            currentLeadIndex = closestIndex;
        }
        else if (currentFollowIndex == -1)
        {
            currentFollowIndex = closestIndex;
        }
        else
        {
            currentLeadIndex = closestIndex;
            currentFollowIndex = -1;
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
            SetDancer(new Vector2((float) x, (float) y));
        }
    }
}