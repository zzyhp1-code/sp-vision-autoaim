#ifndef TOOLS__PLOTTER_HPP
#define TOOLS__PLOTTER_HPP

#include <netinet/in.h>  // sockaddr_in

#include <mutex>
#include <nlohmann/json.hpp>
#include <string>

namespace tools
{
class Plotter
{
public:
  Plotter(std::string host = "127.0.0.1", uint16_t port = 9870);

  ~Plotter();

  // PlotJuggler streaming costs a JSON serialisation plus a UDP send on every
  // frame, and it used to run unconditionally, so disabling the display did not
  // remove it. Turn it off for field runs where nothing is listening.
  void set_enabled(bool enabled) { enabled_ = enabled; }
  bool enabled() const { return enabled_; }

  void plot(const nlohmann::json & json);

private:
  int socket_;
  sockaddr_in destination_;
  std::mutex mutex_;
  bool enabled_ = true;
};

}  // namespace tools

#endif  // TOOLS__PLOTTER_HPP