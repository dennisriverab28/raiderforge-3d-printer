BUILD_DIR := Debug

.PHONY: all clean main-build

all:
	$(MAKE) -C $(BUILD_DIR) all

main-build:
	$(MAKE) -C $(BUILD_DIR) main-build

clean:
	$(MAKE) -C $(BUILD_DIR) clean
