#define CATCH_CONFIG_MAIN  // This tells Catch to provide a main() - only do this once per test-runner!

#include "catch.hpp"
#include "PrimeChecker.hpp"

TEST_CASE("Test PrimeChecker 1.") {
    PrimeChecker pc;
    REQUIRE(pc.isPrime(5));
}