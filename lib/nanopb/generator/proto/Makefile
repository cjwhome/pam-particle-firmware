PROTOC?=protoc

all: nanoMY_pb_pb2.py

%_pb2.py: %.proto
	$(PROTOC) --python_out=. $<

.PHONY: clean
clean:
	rm nanoMY_pb_pb2.py
