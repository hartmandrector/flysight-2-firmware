# tests/Makefile
CC = gcc
CFLAGS = -Wall -Wextra -std=c99 -g -I../FlySight -I../Core/Inc
LDFLAGS = -lm

# Targets
TEST_TIME = test_time
CALC_BLE = calc_ble_dividers

all: $(TEST_TIME) $(CALC_BLE)

# Test time normalization
$(TEST_TIME): test_time.c
	$(CC) $(CFLAGS) -o $(TEST_TIME) test_time.c

# BLE divider calculator
$(CALC_BLE): calc_ble_dividers.c ../FlySight/ble_config.c
	$(CC) $(CFLAGS) -o $(CALC_BLE) calc_ble_dividers.c $(LDFLAGS)

run_test: $(TEST_TIME)
	./$(TEST_TIME)

run_calc: $(CALC_BLE)
	./$(CALC_BLE)

clean:
	rm -f $(TEST_TIME) $(CALC_BLE)

.PHONY: all run_test run_calc clean