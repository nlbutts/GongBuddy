#!/bin/bash
protoc --python_out=. gb_messages.proto
../nanopb/generator/nanopb_generator.py gb_messages.proto
cp gb_messages_pb2.py ../python

