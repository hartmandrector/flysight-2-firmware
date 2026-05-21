# tests/Makefile
CC = gcc
CFLAGS = -Wall -Wextra -std=c99 -g -I../FlySight -I../Core/Inc
LDFLAGS = -lm

# targets
TEST_TIME = test_time

all: $(TEST_TIME)

# Test time normalization
$(TEST_TIME): test_time.c
	$(CC) $(CFLAGS) -o $(TEST_TIME) test_time.c

run_test: $(TEST_TIME)
	./$(TEST_TIME)

clean:
	rm -f $(TEST_TIME)

.PHONY: all run_test clean