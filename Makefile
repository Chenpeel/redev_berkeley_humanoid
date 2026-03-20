UV ?= uv
LOWLEVEL_BUILD_DIR ?= build/lowlevel/native
LOWLEVEL_ARGS ?=
IMU_TEST_ARGS ?=

.PHONY: help verify lint test lock tree clean-pycache
.PHONY: lowlevel-configure lowlevel-build lowlevel-run lowlevel-rebuild lowlevel-clean
.PHONY: lowlevel-build-imu-test lowlevel-run-imu-test lowlevel-build-udp-test lowlevel-run-udp-test

help: ## 显示可用的 make 目标
	@printf "Available targets:\n"
	@awk 'BEGIN {FS = ":.*## "}; /^[a-zA-Z0-9_.-]+:.*## / {printf "  %-38s  %s\n", $$1, $$2}' $(MAKEFILE_LIST)

verify: ## 快速做 Python 语法检查
	$(UV) run python -m compileall tools apps packages/assets/src packages/assets/tests packages/lowlevel/src packages/lowlevel/tests packages/sim/src packages/sim/tests

lint: ## 运行 Ruff 静态检查
	$(UV) run ruff check apps packages tools

test: ## 运行 Python 单元测试
	$(UV) run pytest packages/assets/tests packages/lowlevel/tests packages/sim/tests

lock: ## 更新 uv 锁文件
	$(UV) lock

tree: ## 查看仓库目录树
	find packages apps configs docs tools -maxdepth 2 -type d | sort

clean-pycache: ## 清理 Python 缓存文件
	find . -type d -name __pycache__ -prune -exec rm -rf {} +
	find . -type f \( -name '*.pyc' -o -name '*.pyo' \) -delete

lowlevel-configure: ## 配置 lowlevel native CMake 工程
	cmake -S packages/lowlevel/native -B $(LOWLEVEL_BUILD_DIR)

lowlevel-build: lowlevel-configure ## 编译 lowlevel native main
	cmake --build $(LOWLEVEL_BUILD_DIR) --target main -j

lowlevel-run: lowlevel-build ## 编译并运行 lowlevel native main
	./$(LOWLEVEL_BUILD_DIR)/main $(LOWLEVEL_ARGS)

lowlevel-rebuild: lowlevel-configure ## 清理后重新编译 lowlevel native main
	cmake --build $(LOWLEVEL_BUILD_DIR) --target clean
	cmake --build $(LOWLEVEL_BUILD_DIR) --target main -j

lowlevel-clean: ## 删除 lowlevel native 构建目录
	rm -rf $(LOWLEVEL_BUILD_DIR)

lowlevel-build-imu-test: lowlevel-configure ## 编译 lowlevel native test-imu
	cmake --build $(LOWLEVEL_BUILD_DIR) --target test-imu -j

lowlevel-run-imu-test: lowlevel-build-imu-test ## 编译并运行 lowlevel native test-imu
	./$(LOWLEVEL_BUILD_DIR)/test-imu $(IMU_TEST_ARGS)

lowlevel-build-udp-test: lowlevel-configure ## 编译 lowlevel native test-udp
	cmake --build $(LOWLEVEL_BUILD_DIR) --target test-udp -j

lowlevel-run-udp-test: lowlevel-build-udp-test ## 编译并运行 lowlevel native test-udp
	./$(LOWLEVEL_BUILD_DIR)/test-udp
