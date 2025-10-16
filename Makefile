.PHONY: link-headers clean-headers headers

# Backwards-compatible wrappers that call the unified `headers` target
link-headers:
	@$(MAKE) headers ACTION=link $(if $(EXTERNAL_PATH),EXTERNAL_PATH=$(EXTERNAL_PATH),) $(if $(FORCE),FORCE=$(FORCE),)

clean-headers:
	@$(MAKE) headers ACTION=clean $(if $(FORCE),FORCE=$(FORCE),)

# Unified headers target: ACTION=link (default) or ACTION=clean
# Supports EXTERNAL_PATH and FORCE=1. Prompts interactively unless forced.
headers:
	@act="$(if $(ACTION),$(ACTION),link)"; \
	ext="$(EXTERNAL_PATH)"; \
	work="$$(pwd)"; \
	local_dir="$$work/local_modelspace"; \
	if [ "$$act" = "clean" ]; then \
	  if [ -e "$$local_dir" ]; then \
	    target=$$(readlink -f "$$local_dir" 2>/dev/null || true); \
	    echo "Found $$local_dir -> $$target"; \
	    if [ "$(FORCE)" = "1" ] ; then rm -rf "$$local_dir"; echo "Removed $$local_dir"; else \
	      printf "Remove existing '%s' that points to '%s'? [y/N]: " "$$local_dir" "$$target"; read -r ans || true; \
	      case "$$ans" in y|Y|yes|YES) rm -rf "$$local_dir"; echo "Removed $$local_dir";; *) echo "Aborted by user."; exit 1;; esac; \
	    fi; \
	  else \
	    echo "No existing $$local_dir to remove."; \
	  fi; \
	elif [ "$$act" = "link" ]; then \
	  if [ -n "$$ext" ]; then external_path="$$ext"; else external_path="/usr/include/modelspace"; fi; \
	  if [ ! -e "$$external_path" ]; then echo "Error: external headers path not found: $$external_path" >&2; exit 2; fi; \
	  if [ -e "$$local_dir" ]; then \
	    target=$$(readlink -f "$$local_dir" 2>/dev/null || true); \
	    echo "Found existing $$local_dir -> $$target"; \
	    if [ "$(FORCE)" = "1" ] ; then rm -rf "$$local_dir"; else \
	      printf "Replace existing '%s' that points to '%s'? [y/N]: " "$$local_dir" "$$target"; read -r ans || true; \
	      case "$$ans" in y|Y|yes|YES) rm -rf "$$local_dir";; *) echo "Aborted by user."; exit 1;; esac; \
	    fi; \
	  fi; \
	  ln -s "$$external_path" "$$local_dir"; echo "Created symlink: $$local_dir -> $$external_path"; \
	  touch "$$work/.local_modelspace_index_marker" 2>/dev/null || true; sleep 0.1; rm -f "$$work/.local_modelspace_index_marker" 2>/dev/null || true; \
	else \
	  echo "Unknown ACTION: $$act (valid: link, clean)"; exit 2; \
	fi

# Run full setup: create symlink (accepts EXTERNAL_PATH variable) and nudge indexer
# Usage:
#   make setup                # uses default path (/usr/include/modelspace)
#   make setup EXTERNAL_PATH=/path/to/external
setup:
	@$(MAKE) headers ACTION=clean FORCE=1
	@if [ -n "$(EXTERNAL_PATH)" ]; then \
		$(MAKE) link-headers EXTERNAL_PATH=$(EXTERNAL_PATH) $(if $(FORCE),FORCE=$(FORCE),); \
	else \
		$(MAKE) link-headers $(if $(FORCE),FORCE=$(FORCE),); \
	fi
	@touch .local_modelspace_index_marker 2>/dev/null || true; sleep 0.1; rm -f .local_modelspace_index_marker 2>/dev/null || true
