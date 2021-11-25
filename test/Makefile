CC = gcc
CPP = g++
CFLAGS = -Wall -g -DTEST_FEC -fsanitize=address
LIBCORRECT_OBJS = bit.o convolutional.o decode.o encode.o error_buffer.o history_buffer.o lookup.o \
	fec_shim.o metric.o decode_rs.o encode_rs.o polynomial.o reed-solomon.o
OBJS = test_fec.o fec.o 
INCLUDES = -I../inc/peripherals -I../inc/protocol -I../inc -I../lib/libcorrect/include -I../lib/golay -I./
LIBS = -lm
TARGET = test_fec
LIBCORRECT_PATH = ../lib/libcorrect/


all: $(TARGET)


fec_shim.o : 
	$(CC) $(CFLAGS) -c ../lib/libcorrect/src/fec_shim.c $(INCLUDES) 

bit.o : 
	$(CC) $(CFLAGS) -c ../lib/libcorrect/src/convolutional/bit.c $(INCLUDES) 

convolutional.o : 
	$(CC) $(CFLAGS) -c ../lib/libcorrect/src/convolutional/convolutional.c $(INCLUDES) 

decode.o : 
	$(CC) $(CFLAGS) -c ../lib/libcorrect/src/convolutional/decode.c $(INCLUDES) 

encode.o : 
	$(CC) $(CFLAGS) -c ../lib/libcorrect/src/convolutional/encode.c $(INCLUDES) 

error_buffer.o : 
	$(CC) $(CFLAGS) -c ../lib/libcorrect/src/convolutional/error_buffer.c $(INCLUDES) 

history_buffer.o : 
	$(CC) $(CFLAGS) -c ../lib/libcorrect/src/convolutional/history_buffer.c $(INCLUDES)
	
lookup.o : 
	$(CC) $(CFLAGS) -c ../lib/libcorrect/src/convolutional/lookup.c $(INCLUDES) 

metric.o : 
	$(CC) $(CFLAGS) -c ../lib/libcorrect/src/convolutional/metric.c $(INCLUDES) 

decode_rs.o : 
	$(CC) $(CFLAGS) -c ../lib/libcorrect/src/reed-solomon/decode_rs.c $(INCLUDES) 

encode_rs.o : 
	$(CC) $(CFLAGS) -c ../lib/libcorrect/src/reed-solomon/encode_rs.c $(INCLUDES) 

polynomial.o : 
	$(CC) $(CFLAGS) -c ../lib/libcorrect/src/reed-solomon/polynomial.c $(INCLUDES) 

reed-solomon.o : 
	$(CC) $(CFLAGS) -c ../lib/libcorrect/src/reed-solomon/reed-solomon.c $(INCLUDES) 

test_fec.o : 
	$(CC) $(CFLAGS) -c test_fec.cpp $(INCLUDES) 

fec.o : 
	$(CPP) $(CFLAGS) -c ../src/protocol/fec.cpp $(INCLUDES)

# Golay coding stuff
golay.o :
	$(CC) $(CFLAGS) -c ../lib/golay/golay.c $(INCLUDES) 

test_fec : fec_shim.o bit.o convolutional.o decode.o encode.o error_buffer.o history_buffer.o \
		lookup.o metric.o decode_rs.o encode_rs.o polynomial.o reed-solomon.o test_fec.o \
		fec.o golay.o
	$(CPP) $(CFLAGS) -o test_fec fec_shim.o bit.o convolutional.o decode.o encode.o error_buffer.o \
		history_buffer.o lookup.o metric.o decode_rs.o encode_rs.o polynomial.o reed-solomon.o \
		test_fec.o fec.o golay.o

#$(TARGET): $(TARGET).o
#	$(CC) $(CFLAGS) $(INCLUDES) -o $(TARGET) $(TARGET).c
#	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $^ $(OBJS) $(LIBS)

clean:
	$(RM) $(TARGET) *.o
