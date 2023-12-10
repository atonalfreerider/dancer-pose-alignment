using System.Numerics;
using Avalonia.Controls;
using Avalonia.Input;
using Avalonia.Interactivity;
using Avalonia.Media;
using Avalonia.Threading;
using dancer_pose_alignment;
using Newtonsoft.Json;

namespace GUI;

public partial class MainWindow
{
    CameraPoseSolver cameraPoseSolver;
    int selectedCanvas = -1;

    void LoadJsonForPerspectiveButton_Click(object sender, RoutedEventArgs e)
    {
        string directoryPath = DirectoryPathTextBox.Text;

        if (!Directory.Exists(directoryPath))
        {
            // Handle the case where the directory does not exist
            Console.WriteLine("Directory does not exist.");
            return;
        }

        string sizesJsonPath = Path.Combine(directoryPath, "cameraSizes.json");
        List<Vector2> cameraSizes = JsonConvert.DeserializeObject<List<Vector2>>(File.ReadAllText(sizesJsonPath));

        cameraPoseSolver = new CameraPoseSolver();
        cameraPoseSolver.LoadPoses(directoryPath);

        cameraPoseSolver.HomeAllCameras();

        numCameras = cameraSizes.Count;
        CanvasContainer.Items.Clear();

        for (int i = 0; i < numCameras; i++)
        {
            Image image = new Image();
            alignmentImages.Add(image);
            Canvas canvas = new Canvas();
            canvas.Children.Add(image);
            canvas.Width = cameraSizes[i].X;
            canvas.Height = cameraSizes[i].Y;
            canvas.PointerPressed += Canvas_PointerPressed;
            CanvasContainer.Items.Add(canvas);
        }

        SetPreviewsToFrame();
    }

    void SetPreviewsToFrame()
    {
        float error = cameraPoseSolver.Calculate3DPosesAndTotalError();
        SolverErrorText.Text = error.ToString();

        List<List<Vector2>> reverseProjectedOriginCrossPerCam = cameraPoseSolver.ReverseProjectOriginCrossPerCamera();

        List<List<Vector2>> reverseProjectionsOfOtherCamerasPerCamera = [];
        for (int i = 0; i < numCameras; i++)
        {
            reverseProjectionsOfOtherCamerasPerCamera.Add(cameraPoseSolver
                .ReverseProjectionsOfOtherCamerasPerCamera(i));
        }

        List<List<Vector2>> leadReverseProjectedPerCamera =
            cameraPoseSolver.ReverseProjectionOfLeadPosePerCamera();
        List<List<Vector2>> followReverseProjectedPerCamera =
            cameraPoseSolver.ReverseProjectionOfFollowPosePerCamera();

        SolverFrameNumberText.Text = $"{cameraPoseSolver.GetFrameNumber()}:{cameraPoseSolver.GetTotalFrameCount()}";
        List<List<List<Vector3>>> posesByPersonAtFrameByCamera =
            cameraPoseSolver.AllPosesAtFramePerCamera();
        List<Dictionary<int, List<Vector3>>> posesByPersonAtFrameByCameraDict =
            posesByPersonAtFrameByCamera.Select(list => list.ToDictionary(list.IndexOf, listVec => listVec)).ToList();

        List<List<Vector3>> otherCameraPositionsByFrame = cameraPoseSolver.AllVisibleCamerasAtFramePerCamera();
        List<List<Vector3>> otherMirroredCameraPositionsByFrame =
            cameraPoseSolver.AllMirrorVisibleCamerasAtFramePerCamera();

        for (int i = 0; i < numCameras; i++)
        {
            int i1 = i;
            Dispatcher.UIThread.Post(() =>
            {
                DrawingImage drawingImage = new DrawingImage();
                DrawingGroup drawingGroup = PreviewDrawer.DrawGeometry(
                    posesByPersonAtFrameByCameraDict[i1],
                    alignmentImages[i1].Bounds.Size,
                    0,
                    1,
                    2,
                    3,
                    otherCameraPositionsByFrame[i1],
                    otherMirroredCameraPositionsByFrame[i1],
                    reverseProjectedOriginCrossPerCam[i1],
                    reverseProjectionsOfOtherCamerasPerCamera[i1],
                    leadReverseProjectedPerCamera[i1],
                    followReverseProjectedPerCamera[i1]);
                drawingImage.Drawing = drawingGroup;
                alignmentImages[i1].Source = drawingImage;
            }, DispatcherPriority.Render);
        }

        Dispatcher.UIThread.RunJobs();
    }

    void Canvas_PointerPressed(object sender, PointerPressedEventArgs e)
    {
        // Mark this canvas as selected
        selectedCanvas = CanvasContainer.Items.IndexOf(sender as Canvas);
        Console.WriteLine(selectedCanvas);
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
        SetPreviewsToFrame();
    }

    void SolverPreviousFrameButton_Click(object sender, RoutedEventArgs e)
    {
        if (!cameraPoseSolver.Rewind()) return;
        SetPreviewsToFrame();
    }

    void SolveButton_Click(object sender, RoutedEventArgs e)
    {
        cameraPoseSolver.IterationLoop();
    }

    void Save3D_Click(object sender, RoutedEventArgs e)
    {
        cameraPoseSolver.SaveData();
    }

    
}