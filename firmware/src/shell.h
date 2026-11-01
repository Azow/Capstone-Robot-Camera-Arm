#pragma once

// Bring-up command shell over USB serial: one text command per line, each answered with a
// line starting "OK" or "ERR". Type HELP for the list.
//
// This is for testing the board by hand. The real protocol (D-07: sequence numbers, timed
// segments, queue status) replaces it once it's specified.

namespace shell {

void poll();  // call from loop(): reads input and runs complete lines

}  // namespace shell
