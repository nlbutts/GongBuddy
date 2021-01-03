#!/bin/bash
../nanopb/generator/nanopb_generator.py gb_messages.proto
protoc --python_out=. gb_messages.proto