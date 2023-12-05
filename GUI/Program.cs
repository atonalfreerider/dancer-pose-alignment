using Avalonia;
using Avalonia.Controls;
using Avalonia.Media.Imaging;
using Avalonia.ReactiveUI;
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

    public static void RenderFrame(Args args)
    {
        FrameSource frameSource = Cv2.CreateFrameSource_Video(args.VideoPath);
        OutputArray outputArray = new Mat();
        frameSource.NextFrame(outputArray);
        
        Mat frameMat = outputArray.GetMat();
        Bitmap frame = Bitmap.DecodeToWidth(frameMat.ToMemoryStream(), 1000);
        
        MainWindow.updatePreview.Invoke(frame);
    }
}