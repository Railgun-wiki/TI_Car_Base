#pragma once
namespace drivers {
class ActiveBuzzer final {
public:
  void set(bool on) noexcept;
};
} // namespace drivers
