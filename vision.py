# NOTE: Camera can safely capture at 15 FPS (0.066~ seconds/frame) with sub-
#       millisecond latency per frame. Capturing at higher framerates (e.g. 20
#       FPS) *WILL* result in blocking while the camera buffers the next frame.

# NOTE: Just looping over all pixels can take 100-200ms, an optimized image scan
#       is needed. Multi-threaded/multi-process image processing may not be a
#       terrible idea (barring deadline constraints).

# NOTE: Horizontal FoV is ~60 degrees
