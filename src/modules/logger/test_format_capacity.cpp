// Standalone regression against the build's actual compressed uORB formats.
#include "messages.h"
#include <uORB/uORBMessageFields.hpp>
#include <cassert>
#include <cstdarg>
#include <cstdio>

// Standalone host test: keep decoder diagnostics without starting PX4.
extern "C" void px4_log_modulename(int, const char *module, const char *format, ...) {
  std::fprintf(stderr, "%s: ", module);
  va_list args;
  va_start(args, format);
  std::vfprintf(stderr, format, args);
  va_end(args);
  std::fputc('\n', stderr);
}

int main() {
  ulog_message_format_s message{};
  uORB::MessageFormatReader reader(message.format, sizeof(message.format));
  unsigned count = 0, maximum = 0;
  for (;;) {
    const auto state = reader.readMore();
    assert(state != uORB::MessageFormatReader::State::Failure);
    if (state == uORB::MessageFormatReader::State::Complete) { break; }
    if (state != uORB::MessageFormatReader::State::FormatComplete) { continue; }
    const unsigned size = reader.formatLength();
    const unsigned leftover = reader.moveLeftoverToBufferEnd();
    const int expanded = uORB::MessageFormatReader::expandMessageFormat(message.format, size,
        sizeof(message.format) - leftover);
    assert(expanded > 0);
    // Reserve a 64-character topic name, colon and terminator, including
    // decoder leftovers exactly as Logger::write_formats does.
    assert(unsigned(expanded) + 66 <= sizeof(message.format) - leftover);
    if (unsigned(expanded) > maximum) { maximum = expanded; }
    ++count;
    reader.clearFormatAndRestoreLeftover();
  }
  assert(count > 0);
  printf("ULog format capacity: %u formats, largest expanded fields %u bytes, buffer %zu bytes\n",
      count, maximum, sizeof(message.format));
}
