#ifndef VERSION_H
#define VERSION_H

// When running `pio run` MAJOR, MINOR, and PATCH are set using environment variables
// (see build_flags in platformio.ini). The release workflow sets them from the git tag;
// otherwise they default to 0.0.0.
// These are just set here so we don't get errors showing up in intellisense.
#ifdef __INTELLISENSE__
#define MAJOR_VERSION 0
#define MINOR_VERSION 0
#define PATCH_VERSION 0
#endif

// Check that the versions have been set. An unset environment variable makes the macro expand to
// nothing, so these fail to compile and the comment on the failing line says how to fix it.
// constexpr (not a plain global) so this header stays safe to include from more than one .cpp.
constexpr int _CHECK_MAJOR_SET = MAJOR_VERSION; // Set MAJOR version using `export MAJOR_VERSION=<version>`
constexpr int _CHECK_MINOR_SET = MINOR_VERSION; // Set MINOR version using `export MINOR_VERSION=<version>`
constexpr int _CHECK_PATCH_SET = PATCH_VERSION; // Set PATCH version using `export PATCH_VERSION=<version>`

// The version is packed into a single byte of the periodic status message as
// major*100 + minor*10 + patch. This deliberately trades version range for
// payload space: minor and patch are limited to one digit each, and the packed
// value has to fit in a uint8_t, so the highest encodable version is v2.5.5.
#define FW_VERSION_BYTE (MAJOR_VERSION * 100 + MINOR_VERSION * 10 + PATCH_VERSION)

static_assert(MINOR_VERSION <= 9, "MINOR_VERSION must be 0-9 to pack into the status byte");
static_assert(PATCH_VERSION <= 9, "PATCH_VERSION must be 0-9 to pack into the status byte");
static_assert(FW_VERSION_BYTE <= 255, "Firmware version does not fit in one byte (max is v2.5.5)");

#endif
