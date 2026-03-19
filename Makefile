UV ?= uv

.PHONY: verify lint test lock tree clean-pycache

verify:
	$(UV) run python -m compileall tools apps packages/assets/src packages/assets/tests packages/lowlevel/src packages/lowlevel/tests packages/sim/src packages/sim/tests

lint:
	$(UV) run ruff check apps packages tools

test:
	$(UV) run pytest packages/assets/tests packages/lowlevel/tests packages/sim/tests

lock:
	$(UV) lock

tree:
	find packages apps configs docs tools -maxdepth 2 -type d | sort

clean-pycache:
	find . -type d -name __pycache__ -prune -exec rm -rf {} +
	find . -type f \( -name '*.pyc' -o -name '*.pyo' \) -delete
