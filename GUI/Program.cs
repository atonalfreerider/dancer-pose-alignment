using System.Numerics;
using Avalonia;
using Avalonia.Controls;
using Avalonia.Media.Imaging;
using Avalonia.ReactiveUI;
using dancer_pose_alignment;
using OpenCvSharp;

namespace GUI;

static class Program
{
    public class Args
    {
        public string VideoPath { get; init; }
        
        public string AlphaPoseJsonPath { get; init; }
    }

    // This method is needed for IDE previewer infrastructure
    static AppBuilder BuildAvaloniaApp() => AppBuilder.Configure<App>()
        .UsePlatformDetect()
        .UseReactiveUI();

    // The entry point. Things aren't ready yet, so at this point you shouldn't use any Avalonia types or anything
    // that expects a SynchronizationContext to be ready
    public static void Main(string[] args) => BuildAvaloniaApp()
        .Start(AppMain, args);

    static MainWindow mainWindow;

    // Application entry point. Avalonia is completely initialized.
    static void AppMain(Application app, string[] args)
    {
        // A cancellation token source that will be used to stop the main loop
        CancellationTokenSource cts = new CancellationTokenSource();

        // Do you startup code here
        mainWindow = new MainWindow();
        mainWindow.Show();
        
        MainWindow.SetupPreview();

        // Start the main loop
        app.Run(cts.Token);
    }
    
    static bool hasBeenInitialized = false; 
    static FrameSource frameSource;
    static Dictionary<int, Dictionary<int, List<Vector3>>> PosesByFrameByPerson;
    static int frameCount = 0;
    
    public static void RenderFrame(Args args)
    {
        string videoPath = args.VideoPath;
        string alphaPoseJsonPath = args.AlphaPoseJsonPath;
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

        Dictionary<int, List<Vector3>> posesByPersonAtFrame = new();
        foreach ((int personId, Dictionary<int, List<Vector3>> posesByFrame) in PosesByFrameByPerson)
        {
            if (posesByFrame.ContainsKey(frameCount))
            {
                posesByPersonAtFrame.Add(personId, posesByFrame[frameCount]);
            }
        }
        
        MainWindow.updatePreview.Invoke(frame, posesByPersonAtFrame);

        frameCount++;
    }
}