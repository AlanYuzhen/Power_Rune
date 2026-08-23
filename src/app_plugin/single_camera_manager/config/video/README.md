# Demo video asset

This document accompanies the replay asset owned by the `single_camera_manager`
plugin.

## Technical properties

- Path: `src/app_plugin/single_camera_manager/config/video/big_rune_demo.mp4`
- Source filename: `big_rune.avi`
- Content: 30-second demonstration excerpt supplied by the maintainers
- Codec: H.264
- Resolution: 1440x1080
- Frame rate: 30 FPS
- Frame count: 900
- SHA-256: `7bfd290058477f7d415d76e4943899061a9e3b15bab3740c2db776b7bbbc6025`

OpenCV must have an FFmpeg or GStreamer backend capable of decoding H.264.
Failure to provide such a backend will cause `CameraManager` to reject the
video at startup.

The resolution must remain consistent with the adjacent `camera.json` and the
camera intrinsics in the repository-level `config/power_rune.json`. If the
video is replaced, update those settings and this checksum together.

## Provenance and release rights

The video is a 30-second excerpt made from the maintainer-provided source
recording `big_rune.avi`. The excerpt is distributed under the repository's
root MIT License.

Before publishing the repository, its maintainers must confirm that they own
or have permission to redistribute the original recording and all third-party
material it may depict. Applying the repository's MIT License to the excerpt
does not clear independent rights belonging to people, venues, brands,
competition organizers, music owners, or other third parties.
