// Link stubs for pieces of the host platform the drift tests don't exercise.
#include <sys/select.h>
#include <cstdint>
namespace esphome {
void wake_loop_threadsafe() {}
void wake_setup() {}
namespace internal {
int g_wake_socket_fd = -1;
fd_set g_read_fds;
void wakeable_delay(uint32_t) {}
}  // namespace internal
namespace host {
char** get_argv() { return nullptr; }
const char* get_reexec_path() { return nullptr; }
}  // namespace host
}  // namespace esphome
void setup() {}
void loop() {}
