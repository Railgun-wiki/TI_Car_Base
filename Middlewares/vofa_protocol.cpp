#include "Middlewares/vofa_protocol.hpp"

#include <cstring>

namespace middleware {
namespace {
bool parseFloat(const char *&cursor, float &value) noexcept {
  bool negative = false;
  if (*cursor == '+' || *cursor == '-') {
    negative = *cursor == '-';
    ++cursor;
  }
  if (*cursor < '0' || *cursor > '9')
    return false;

  float result = 0.0F;
  while (*cursor >= '0' && *cursor <= '9') {
    result = result * 10.0F + static_cast<float>(*cursor - '0');
    ++cursor;
  }
  if (*cursor == '.') {
    ++cursor;
    float scale = 0.1F;
    if (*cursor < '0' || *cursor > '9')
      return false;
    while (*cursor >= '0' && *cursor <= '9') {
      result += static_cast<float>(*cursor - '0') * scale;
      scale *= 0.1F;
      ++cursor;
    }
  }
  value = negative ? -result : result;
  return true;
}

bool parseInt(const char *&cursor, int &value) noexcept {
  if (*cursor < '0' || *cursor > '9')
    return false;
  int result = 0;
  while (*cursor >= '0' && *cursor <= '9') {
    if (result > 3276)
      return false;
    result = result * 10 + (*cursor - '0');
    ++cursor;
  }
  value = result;
  return true;
}

VofaCommand parse(const char *line) noexcept {
  VofaCommand command{};

  if (std::strcmp(line, "GET,CONFIG") == 0) {
    command.type = VofaCommandType::GetConfig;
  } else {
    static constexpr char kLinePrefix[] = "SET,LINE,";
    static constexpr char kCruisePrefix[] = "SET,CRUISE,";
    if (std::strncmp(line, kLinePrefix, sizeof(kLinePrefix) - 1U) == 0) {
      const char *cursor = line + sizeof(kLinePrefix) - 1U;
      float kp = 0.0F;
      float ki = 0.0F;
      float kd = 0.0F;
      if (parseFloat(cursor, kp) && *cursor++ == ',' &&
          parseFloat(cursor, ki) && *cursor++ == ',' &&
          parseFloat(cursor, kd) && *cursor == '\0')
        command = {VofaCommandType::SetLinePid, kp, ki, kd, 0};
      else
        command.type = VofaCommandType::Invalid;
    } else if (std::strncmp(line, kCruisePrefix, sizeof(kCruisePrefix) - 1U) ==
               0) {
      const char *cursor = line + sizeof(kCruisePrefix) - 1U;
      int cruise = 0;
      if (parseInt(cursor, cruise) && *cursor == '\0' && cruise <= 32767) {
        command.type = VofaCommandType::SetCruise;
        command.cruise = static_cast<std::int16_t>(cruise);
      } else {
        command.type = VofaCommandType::Invalid;
      }
    } else {
      command.type = VofaCommandType::Invalid;
    }
  }
  return command;
}
} // namespace

bool VofaProtocol::consume(std::uint8_t byte, VofaCommand &command) noexcept {
  command = {};
  if (byte == '\r')
    return false;
  if (byte != '\n') {
    if (length_ + 1U < kLineCapacity)
      line_[length_++] = static_cast<char>(byte);
    else
      overflow_ = true;
    return false;
  }

  if (overflow_ || length_ == 0U) {
    command.type = VofaCommandType::Invalid;
  } else {
    line_[length_] = '\0';
    command = parse(line_);
  }
  length_ = 0U;
  overflow_ = false;
  return true;
}

} // namespace middleware
