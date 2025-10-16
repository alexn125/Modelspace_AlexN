Adding external modelspace headers to Quick Open (Ctrl+P)

Goal
----
Make header files located outside the repository (for example, /usr/include/modelspace) discoverable in editors' Quick Open (Ctrl+P) without committing external files into the repo.

Quick solution
--------------
1. Run the command below
```bash
make setup															# uses default /usr/include/modelspace
make setup EXTERNAL_PATH=/path/to/external/usr/include/modelspace	# or specify an external path:
```
2. May need to reload the window. To do this search command (Ctrl+Shift+P) "Developer Reload Window"

Notes for collaborators
----------------------
- Everyone who clones the repo should run the script locally and point it at their system's headers location. The script will fail early if the path doesn't exist.
- The symlink target is not stored in git; only the local symlink is created. This avoids repository pollution while providing a reproducible helper.

Troubleshooting
---------------
- If Quick Open doesn't show files after creating the symlink, try restarting the editor or manually adding the folder to your workspace view so it gets indexed.
- On some systems the headers may be under a different path (for example `/usr/local/include/modelspace` or a distro-specific prefix). Locate them first with `find /usr -type d -name modelspace`.
 - VS Code specifics that help Quick Open index symlinked folders:
	 1. Make sure search follows symlinks. In VS Code settings this is `search.followSymlinks: true` (the workspace `settings.json` in this repo sets that by default).
	 2. Ensure the folder is not excluded in `search.exclude` or `files.exclude`. This workspace sets `local_modelspace` to not be excluded.
	 3. Force a reindex by running the command palette action: "Developer: Reindex Workspace" (open with Ctrl+Shift+P and type the command). If you don't see that command, run "Developer: Reload Window" then try reindex.
		4. If Quick Open still hides the files, try briefly opening the `local_modelspace` folder in the Explorer view (right-click the folder -> "Reveal in Explorer" or add folder to workspace). Opening a few files once forces them into the Quick Open cache on some versions of VS Code.
		  5. The helper creates a short-lived marker file in the repo root (named `.local_modelspace_index_marker`) after creating the symlink to nudge file watchers. This avoids touching files inside system folders like `/usr/include` where you may not have write permission.
