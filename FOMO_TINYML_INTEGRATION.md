# FOMO / TinyML integration

This revision adds a real **Edge Impulse FOMO inference adapter** to the SAR controller.
It is intentionally not shipped with a fabricated neural-network model: a useful SAR
model must be trained on real images from the actual OV7670 and environment.

## What the code does

1. Captures the OV7670 frame at 160x120.
2. Resamples it to 96x96 grayscale for the TinyML model.
3. Calls the Edge Impulse `run_classifier()` API when the exported library is present.
4. Searches FOMO object detections for `person`, `human`, or `survivor`.
5. Applies a 0.65 minimum confidence threshold.
6. Converts the FOMO centroid to normalized camera coordinates.
7. Uses the FOMO centroid for the existing camera/IMU geolocation correction.
8. Passes the neural candidate into the existing multi-frame + MLX90640 confirmation logic.
9. Reports inference time, confidence, centroid and inference count through telemetry.
10. If the Edge Impulse library is absent, the controller compiles and safely falls back
to the existing non-AI visual candidate detector.

## Required model

Train a FOMO object-detection model with a class named **person** (or human/survivor),
preferably using images captured with the same OV7670, lens, mounting angle and expected
search altitude. A practical first target is 96x96 grayscale and an int8/quantized model.

Export the trained model as an **Arduino library** from Edge Impulse and copy its
`edge-impulse-sdk/` directory next to the `.ino` sketch. The library must contain:

`edge-impulse-sdk/classifier/ei_run_classifier.h`

The code automatically detects this header with `__has_include()`.

## Important model/data requirement

The neural network cannot be honestly pre-trained for your SAR application without a
representative dataset. Do not interpret the fallback detector as AI. Until the exported
FOMO library is installed, telemetry will show:

`FOMO: library=0`

After the library is installed and working, it should show:

`FOMO: library=1 healthy=1`

The exact inference time depends on the ESP32 variant, compiler, model architecture,
input size, camera driver and PSRAM configuration. Measure it on the target board rather
than assuming a fixed FPS.

## Recommended training dataset

Collect and label:

- people standing and lying down
- partially occluded people
- people in mud, vegetation and rubble
- different clothing colours
- daylight, shade and low-light conditions
- different camera heights and viewing angles
- empty ground and common false positives

For the thermal confirmation stage, keep the MLX90640 independent from the visible CNN.
A visible FOMO detection alone should not trigger the rescue SMS; the existing multi-frame
and thermal confirmation logic remains in the decision path.

## Safety

`BENCH_TEST_MODE` remains enabled. Neural detections must be validated on the ground before
using them for autonomous flight or rescue decisions.
