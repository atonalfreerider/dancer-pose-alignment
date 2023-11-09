# dancer-pose-alignment

take yolov8 2d poses and project them into space to make 3D model

requires yolo8x-pose.onnx.

download from: https://docs.ultralytics.com/tasks/pose/
convert with:

```
from ultralytics import YOLO

# Load a model
model = YOLO('yolov8x-pose.pt')  # load an official model

# Export the model
model.export(format='onnx')
```

requires cameras to be positioned. optionally use https://github.com/atonalfreerider/vr-photogrammetry-placement to calibrate

the unified model can be animated in https://github.com/atonalfreerider/head-movement 