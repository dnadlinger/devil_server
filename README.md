EVIL Network Server
===================

This repository contains a server program offering control
of multiple EVIL PID controllers over the network. It is
intended to run on a mini-computer such as the Raspberry Pi
that is connected directly to the EVIL boxes to offer an
Ethernet interface to them.

This project is written in "modern" C++ and will require a
compiler that supports C++11 to build.

Originally written by David Nadlinger [0] for hardware
developed by Ludwig de Clerq [1] and Vlad Negnevitsky [2]
in the Trapped Ion Quantum Information Group, ETH Zurich.


Documentation
-------------

The code base includes documentation comments compatible
with Doxygen. Run `doxygen` in the root directory of the
repository and then browse to `docs/html/index.html` to
view the generated documentation.


Coding Style
------------

Apart from generally following good programming practice,
please try to keep the code format reasonably consistent.
Sine there is is no excuse for wasting brain-power on that
in this day and age, clang-format [4] is used. Ideally, you
run it before every check-in like this:

    clang-format -style=file -i <files_you_changed>

There are also various options available for editor/IDE
integration.

- - -

[0] mailto:nadavid@phys.ethz.ch
[1] mailto:ludwigd@phys.ethz.ch
[2] mailto:nvlad@phys.ethz.ch
[3] http://clang.llvm.org/docs/ClangFormat.html
