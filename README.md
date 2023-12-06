# dancer-pose-alignment

Input AlphaPose tracked poses and project them into space to make a 3D model

AlphaPose:  
https://github.com/MVIG-SJTU/AlphaPose

conda env create --file environment.yml
conda activate alphapose

config downloaded from here:
https://github.com/MVIG-SJTU/AlphaPose/blob/master/configs/halpe_coco_wholebody_136/resnet/256x192_res50_lr1e-3_2x-regression.yaml

put this in pretrained_models
multi_domain_fast50_regression_256x192.pth
https://drive.google.com/file/d/1Bb3kPoFFt-M0Y3ceqNO8DTXi1iNDd4gI/view?usp=sharing

put this in tracker/weights
osnet_ain_x1_0_msmt17_256x128_amsgrad_ep50_lr0.0015_coslr_b64_fb10_softmax_labsmth_flip_jitter.pth
https://drive.google.com/file/d/1myNKfr2cXqiHZVXaaG8ZAq_U2UpeOLfG/view?usp=share_link
NOTE: filename contains extra "... (1).pth" at end. rename

Data preparation:
- conform .mp4 videos to synchronized start time. currently AlphaPose can only handle 20-30 second clips
- conform framerates to 30fps
- run Alphapose shell with MiniConda: 
- `sh inference-no-img-no-vid.sh ${CONFIG} ${CHECKPOINT} ${VIDEO_NAME} ${OUTPUT_DIR}`
- eg `sh inference-no-img-no-vid.sh "path\AlphaPose\configs\256x192_res50_lr1e-3_2x-regression.yaml" "path\AlphaPose\pretrained_models\multi_domain_fast50_regression_256x192.pth" "path-to-video-0.mp4"  "path-to-output-folder"`

Run the GUI and input each video path and alpha-pose.json. The pose of the leader is indicated by clicking once, the
pose of the follow is indicated by clicking a second time. The poses can be cleared.

`Run Until Next` will fast forward the poses and video until their is a discontinuity in the tracked figure's pose.
The user must manually fix, and continue.

Save the poses.

Run the CameraSolver with the saved lead-{camera number}.json and follow-{camera number}.json files

This will output a set of 3d poses json.

The unified model can be animated in https://github.com/atonalfreerider/head-movement 