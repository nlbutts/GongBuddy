#!/bin/bash
echo "Running protoc"
protoc --python_out=. gb_messages.proto
echo "Running nanopb_generator"
../nanopb/generator/nanopb_generator.py gb_messages.proto
cp gb_messages_pb2.py ../python

