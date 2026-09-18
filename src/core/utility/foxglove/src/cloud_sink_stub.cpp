// Stub implementation of the foxglove cloud sink C API.
//
// The upstream foxglove-sdk prebuilt releases (v0.25.x-v0.27.x) do not ship
// this API, while this repository's vendored wrapper references it. The cloud
// sink path is not exercised by any code in this project, so these stubs only
// provide the symbols required for the shared library to load correctly.
//
// If cloud sink support is ever needed, replace this file with the real
// implementation from a foxglove-sdk build that provides the API.

#include <foxglove-c/foxglove-c.h>

extern "C" {

foxglove_error foxglove_cloud_sink_start(
  const struct foxglove_cloud_sink_options* options, struct foxglove_cloud_sink** server) {
  (void)options;
  if (server != nullptr) {
    *server = nullptr;
  }
  return FOXGLOVE_ERROR_UNSPECIFIED;
}

foxglove_error foxglove_cloud_sink_stop(struct foxglove_cloud_sink* sink) {
  (void)sink;
  return FOXGLOVE_ERROR_OK;
}

}  // extern "C"
