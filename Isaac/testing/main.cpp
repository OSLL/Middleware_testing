#include <string>
#include <vector>

#include "engine/alice/components/alice_all_components.hpp"
#include "engine/alice/tools/parse_command_line.hpp"
#include "engine/core/time.hpp"
#include "gflags/gflags.h"

DEFINE_string(max_duration, "", "Max Duration to run the app for. <number>[s|m|h]");

// A default main file for an Isaac application. This can be used to execute an application defined
// in a JSON file.
int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  ::isaac::alice::Application app(isaac::alice::ParseApplicationCommandLine());

  app.enableStopOnNode("control");
  app.runBlocking();
  return 0;
}
