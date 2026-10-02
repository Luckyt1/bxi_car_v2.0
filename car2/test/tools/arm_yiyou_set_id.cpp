#include <iostream>
#include <string>

namespace
{
void usage()
{
  std::cout <<
    "Usage: arm_yiyou_set_id --help\n"
    "\n"
    "EtherCAT migration note:\n"
    "  Yiyou arm motors are selected by physical EtherCAT chain position with\n"
    "  --interface NIC --slave N or --slaves A,B,C. The old CAN Node-ID rewrite\n"
    "  utility is retired and no longer writes object 0x26A0.\n"
    "\n"
    "Use arm_yiyou_test --interface NIC --slave N --info-only to read identity and\n"
    "confirm the physical position before configuring zero or motion tests.\n"
    "This program never opens CAN, EtherCAT, or motor power.\n";
}

bool has_retired_option(int argc, char ** argv)
{
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--bus" || arg == "--node" || arg == "--nodes" || arg == "--new-id") {
      return true;
    }
  }
  return false;
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc == 2 && std::string(argv[1]) == "--help") {
    usage();
    return 0;
  }
  if (has_retired_option(argc, argv)) {
    std::cerr <<
      "arm_yiyou_set_id: CAN Node-ID changes are retired for EtherCAT arm motors.\n"
      "Use physical chain positions with --interface/--slave in arm_yiyou_test or\n"
      "arm_yiyou_control; object 0x26A0 is not written.\n";
  } else {
    std::cerr << "arm_yiyou_set_id: retired; run --help for the EtherCAT workflow.\n";
  }
  return 2;
}
