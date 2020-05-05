// Include the header-only libcluon file.
#include "cluon-complete.hpp"

// Include our self-contained message definitions.
#include "example.hpp"
#include "opendlv-standard-message-set.hpp"

// We will use iostream to print the received message.
#include <iostream>

int main(int argc, char **argv) {
  if (2 != argc) {
    std::cerr << "Use: " << argv[0] << " example.rec" << std::endl;
    exit(-1);
  }

  // We will use cluon::Player to parse and process a .rec file.
  constexpr bool AUTO_REWIND{false};
  constexpr bool THREADING{false};
  cluon::Player player(std::string(argv[1]), AUTO_REWIND, THREADING);

  // Now, we simply loop over the entries.
  while (player.hasMoreData()) {
    auto entry = player.getNextEnvelopeToBeReplayed();
    // The .first field indicates whether we have a valid entry.
    if (entry.first) {
      // Get the Envelope with the meta-information.
      cluon::data::Envelope env = entry.second;
      std::cout << "Envelope ID/senderStamp = " << env.dataType() << "/" << env.senderStamp() << std::endl
                << " - sent                 = " << env.sent().seconds() << "." << env.sent().microseconds() << std::endl
                << " - received             = " << env.received().seconds() << "." << env.received().microseconds() << std::endl
                << " - sample time          = " << env.sampleTimeStamp().seconds() << "." << env.sampleTimeStamp().microseconds() << std::endl;

      // Check whether it is of type GeodeticWgs84Reading.
      if (env.dataType() == opendlv::proxy::GeodeticWgs84Reading::ID()) {
        // Unpack the content...
        auto msg = cluon::extractMessage<opendlv::proxy::GeodeticWgs84Reading>(std::move(env));

        // ...and access the data fields.
        std::cout << "lat = " << msg.latitude() << ", lon = " << msg.longitude() << std::endl;
      }

      // Check whether it is of type TestMessage2.
      if (env.dataType() == odcore::testdata::TestMessage2::ID()) {
        // Unpack the content...
        auto msg = cluon::extractMessage<odcore::testdata::TestMessage2>(std::move(env));

        // ...and traverse its data fields using a lambda function as alternative.
        msg.accept([](uint32_t, const std::string &, const std::string &) {},
                   [](uint32_t, std::string &&, std::string &&n, auto v) { std::cout << n << " = " << +v << std::endl; },
                   [](){});
      }

      // Check whether it is of type TestMessage5.
      if (env.dataType() == odcore::testdata::TestMessage5::ID()) {
        auto msg = cluon::extractMessage<odcore::testdata::TestMessage5>(std::move(env));
        msg.accept([](uint32_t, const std::string &, const std::string &) {},
                   [](uint32_t, std::string &&, std::string &&n, auto v) { std::cout << n << " = " << v << std::endl; },
                   [](){});
      }
    }
  }

  return 0;
}
