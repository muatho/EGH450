# EXP-009 F1 OAK-D comparison capture

## Identity verified onboard

- Checkpoint named by the deployment manifest:
  `EXP-009_F1/attempt_001/training/weights/last.pt`
- Expected checkpoint SHA-256:
  `ae81b4fd67a68b6a12c254b24ebf5743584da2361942479c30ab4d2eb78e76c7`
- Blob SHA-256:
  `ca9501466f53cf7ae8efa4a849dd032d50e1de405b29702c96a46bfd5fcf537c`
- JSON SHA-256:
  `5387e0cfb2bcccc75f70b9623ec2df5499c1dec5a3919782e63db10ae8928b7e`
- DepthAI Python: `2.25.1.0`
- Blob OpenVINO version: `2022.1`
- Blob resources: 6 SHAVEs, 6 slices, FP16 outputs

The checkpoint itself is not present onboard. Its SHA-256 must be checked on
the Mac before claiming that it is identical to the checkpoint named above.

## Network and decoder configuration

- Camera preview: 640 x 360 at 8 FPS
- Network input: 416 x 416
- Resize: DepthAI `setResizeThumbnail()`, preserving 16:9 content and padding
  it to a square. This gives 416 x 234 image content with 91 rows of vertical
  padding above and below.
- Pipeline colour order and layout: BGR, planar
- Published diagnostic image: `bgr8`, interleaved ROS storage, reconstructed
  from the exact `network.passthrough` packet
- Blob input metadata: NCHW, `U8F`, dimensions `[416, 416, 3, 1]`
- Deployment-manifest expectation: RGB planar with division by 255 embedded
- Confidence threshold: 0.50
- NMS IoU threshold: 0.50
- Classes: 2 (`0 = person`, `1 = backpack`)
- Coordinates: 4
- Anchors: `[10,13, 16,30, 33,23, 30,61, 62,45, 59,119, 116,90,
  156,198, 373,326]`
- Anchor masks: `side52=[0,1,2]`, `side26=[3,4,5]`,
  `side13=[6,7,8]`

The verified BGR pipeline setting conflicts with the manifest's stated RGB
input expectation. Do not correct it before this capture: the comparison is
intended to determine whether it explains the deployment discrepancy.

## Diagnostic topics

- `/depthai_node/nn_input`: exact resized/padded, unannotated network input
- `/yolo/raw_detections`: decoded/NMS detections using source class IDs and
  416 x 416 input-pixel box coordinates
- `/yolo/detections`: normal application output after class handling and
  conversion back to 640 x 360 coordinates
- `/depthai_node/image/raw`: original 640 x 360 BGR preview
- `/depthai_node/camera/camera_info`: calibration for the 640 x 360 preview

Pair `/depthai_node/nn_input` and `/yolo/raw_detections` using `header.seq`.
Their corresponding timestamps are copied from the same source camera-frame
sequence. CameraInfo describes the original preview; it does not directly
describe the padded 416 x 416 diagnostic image.

## Capture procedure

The ordinary launch default currently selects the COCO V2 blob, not EXP-009.
Launch the image pipeline with explicit EXP-009 paths and diagnostics enabled:

```bash
roslaunch depthai_publisher image_topics.launch \
  publish_nn_diagnostics:=true \
  yolo_config_path:=/home/uavteam6/catkin_ws_G6/src/depthai_publisher/src/depthai_publisher/models/egh450_yolo5_exp009_f1/candidate.json \
  yolo_blob_path:=/home/uavteam6/catkin_ws_G6/src/depthai_publisher/src/depthai_publisher/models/egh450_yolo5_exp009_f1/candidate.blob
```

In another terminal, record 60 seconds:

```bash
rosbag record --lz4 --duration=60 -b 512 \
  -O "$CAPTURE_DIR/exp009_oakd_compare" \
  /depthai_node/nn_input \
  /yolo/raw_detections \
  /yolo/detections \
  /depthai_node/image/raw \
  /depthai_node/camera/camera_info \
  /rosout
```

Include empty background, the known false-positive view, a real person and a
real backpack without changing the model, JSON or thresholds during recording.
