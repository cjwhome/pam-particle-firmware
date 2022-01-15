@echo off
:: This file is used to invoke nanoMY_pb_generator.py as a plugin
:: to protoc on Windows.
:: Use it like this:
:: protoc --plugin=protoc-gen-nanopb=..../protoc-gen-nanopb.bat --nanoMY_pb_out=dir foo.proto
::
:: Note that if you use the binary package of nanopb, the protoc
:: path is already set up properly and there is no need to give
:: --plugin= on the command line.

set mydir=%~dp0
python "%mydir%\nanoMY_pb_generator.py" --protoc-plugin %*
