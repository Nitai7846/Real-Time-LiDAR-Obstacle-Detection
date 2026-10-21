# Limitations and follow-up ideas

- **Geometric detection, not semantic recognition.** Clusters do not identify cars/pedestrians/cyclists. No learned features, annotations, trained weights, classwise precision/recall or mAP.
- **Offline input.** Reads `.bin` captures; no physical LiDAR/ROS subscription, timestamp synchronization, sensor driver, or verified real-time deadline for live input.
- **Not necessarily temporally contiguous.** KITTI object-detection training frame numbering is not a continuous drive; animation is sequential **example** playback. No persistent obstacle IDs or trajectory estimation.
- **Single dominant plane.** 3D RANSAC may fail on uneven roads, curbs, slopes beyond the allowed tilt, or multiple height surfaces. Non-ground handling on plane failure is a fallback, not semantic correctness.
- **Distance-based clustering.** Close objects may merge; sparse or distant objects may split; minimum/maximum cluster size filters introduce omissions.
- **Axis-aligned boxes.** AABBs are sensitive to object orientation and partial visible surfaces; not oriented physical-object extent estimates.
- **ROI/downsampling.** Fixed crop limits exclude distant obstacles; voxels discard detail; parameters are tuned for this demonstration, not universally valid.
- **Reference comparability.** PCL and custom plane solvers use distinct sampling/model internals; equal cluster counts and overlap do not imply accuracy against ground truth. PCL is measured faster in the provided comparison.
- **Benchmark scope.** Reported latencies are a single run on the author's Mac; interactive rendering, dataset I/O, custom algorithms, benchmarks and observed wall throughput have different timing boundaries. The GUI target `--fps` is not evidence of achieved FPS.

Potential **future** work, deliberately outside this Milestone 12 release: oriented boxes, object association/tracking, adaptive thresholds/multi-plane ground segmentation, native sensor/ROS integration, independent semantic-label evaluation, better I/O scheduling, and well-controlled repeated performance studies. No existing perception algorithms were modified for this release.
