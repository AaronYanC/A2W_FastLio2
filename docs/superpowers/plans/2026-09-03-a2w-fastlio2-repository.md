# A2W Fast-LIO2 Repository Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Preserve the current runnable JT128 FAST-LIO2 workspace, then turn it into a portable ROS 2 Humble project and push both commits to the private GitHub repository.

**Architecture:** The workspace root becomes an independent Git repository while Hesai FAST-LIO stays pinned as a recursive submodule. A small ament bringup package owns portable JT128 parameters and launch behavior; root-relative shell entry points generate the per-run CycloneDDS XML and pass the map output path at launch time.

**Tech Stack:** Bash, Python 3 standard library, ROS 2 Humble launch/ament_cmake, CycloneDDS, colcon, rosdep, Git submodules

**Spec:** `docs/superpowers/specs/2026-09-03-a2w-fastlio2-repository-design.md`

## Global Constraints

- Support Ubuntu 22.04, ROS 2 Humble, x86_64, and `rmw_cyclonedds_cpp`.
- Work only on the PC-side repository; never log in to or modify robot hosts, networking, LiDAR parameters, drivers, or services.
- Keep generated maps, build products, logs, personal agent state, and credentials out of Git.
- Keep Hesai FAST_LIO_Hesai on its current verified ROS2 commit and do not edit its tracked files.
- The final tracked tree must contain no fixed user home path or legacy project path.
- Do not force-push the GitHub repository.

---

### Task 1: Establish the Independent Repository and Backup Commit

**Files:**
- Create: `.gitmodules`
- Git metadata: workspace-root `.git/`
- Track in backup: existing `config/`, existing scripts, existing tests, existing historical design documents, and `src/FAST_LIO_Hesai` gitlink

**Interfaces:**
- Consumes: clean `src/FAST_LIO_Hesai` checkout on commit `16e97cd`
- Produces: independent `main` branch with one backup commit and official recursive submodule metadata

- [ ] **Step 1: Verify repository and submodule boundaries**

Run:

```bash
git -C src/FAST_LIO_Hesai status --short
git -C src/FAST_LIO_Hesai rev-parse --short HEAD
git -C src/FAST_LIO_Hesai submodule status --recursive
```

Expected: no FAST_LIO changes, HEAD `16e97cd`, and initialized ikd-Tree.

- [ ] **Step 2: Initialize the workspace root as its own repository**

Make the existing empty `.git` directory writable, initialize `main`, set repository-local author identity, and add the SSH-443 remote:

```bash
chmod u+w .git
git init --initial-branch=main .
git config user.name AaronYanC
git config user.email AaronYanC@users.noreply.github.com
git remote add origin ssh://git@ssh.github.com:443/AaronYanC/A2W_FastLio2.git
```

- [ ] **Step 3: Add official submodule metadata**

Create `.gitmodules` with:

```ini
[submodule "src/FAST_LIO_Hesai"]
    path = src/FAST_LIO_Hesai
    url = https://github.com/HesaiTechnology-Spatial-Perception/FAST_LIO_Hesai.git
    branch = ROS2
```

- [ ] **Step 4: Stage only the pre-engineering snapshot**

Explicitly stage `.gitmodules`, the existing two configs, two launch scripts, two legacy tests, two legacy design documents, and the FAST_LIO gitlink. Do not stage the newly approved repository spec/plan, generated directories, maps, or personal directories.

- [ ] **Step 5: Inspect and commit the backup**

Run `git diff --cached --stat`, `git diff --cached --submodule=short`, and `git status --short`, then commit:

```bash
git commit -m "backup: preserve runnable JT128 FAST-LIO2 workspace"
```

Expected: the first commit contains only the intended snapshot and gitlink.

### Task 2: Specify Portable Runtime Behavior With Failing Tests

**Files:**
- Create: `tests/test_runtime_config.py`
- Create: `tests/test_portable_launchers.sh`
- Create: `tests/test_repository_hygiene.sh`
- Create: `tests/run_tests.sh`

**Interfaces:**
- Consumes: environment overrides `A2W_PC_IP`, `A2W_NETWORK_INTERFACE`, `A2W_DDS_PEER`, `A2W_MAP_FILE`, `A2W_ROS_SETUP`, `A2W_INSTALL_SETUP`, and `ROS2_BIN`
- Produces: executable behavioral contract for the runtime XML generator and root-relative launch entry points

- [ ] **Step 1: Write the runtime XML failing test**

Use Python `unittest` to execute `scripts/generate_cyclonedds_config.py` with literal addresses `192.168.123.77` and `192.168.123.164`, parse the result through `xml.etree.ElementTree`, and assert the emitted interface and peer values. Add invalid-address coverage that expects a non-zero exit code and no output file.

- [ ] **Step 2: Run the runtime test and verify RED**

Run:

```bash
python3 -m unittest tests/test_runtime_config.py -v
```

Expected: FAIL because `scripts/generate_cyclonedds_config.py` does not exist.

- [ ] **Step 3: Write the portable launcher failing test**

The Bash test creates temporary empty ROS setup files and a fake `ros2` executable, invokes each real launcher from `/tmp`, and asserts observable output captured by the fake command:

```text
launch a2w_fastlio2_bringup jt128_mapping.launch.py save_map:=false
launch a2w_fastlio2_bringup jt128_mapping.launch.py save_map:=true map_file:=$A2W_FASTLIO_ROOT/maps/jt128_map.pcd
```

It also parses the generated CycloneDDS file and checks that `CYCLONEDDS_URI` and `ROS_LOG_DIR` are rooted in the actual workspace.

- [ ] **Step 4: Run the launcher test and verify RED**

Run `bash tests/test_portable_launchers.sh`.

Expected: FAIL because the new bringup command and runtime config generation are not implemented.

- [ ] **Step 5: Write the hygiene failing test and verify RED**

The test uses `git ls-files -z` and scans current tracked regular files, excluding the FAST_LIO gitlink, for fixed home paths and legacy project names. It must fail before cleanup because the backup files still contain such paths.

### Task 3: Implement Portable Bringup and Launchers

**Files:**
- Create: `src/a2w_fastlio2_bringup/CMakeLists.txt`
- Create: `src/a2w_fastlio2_bringup/package.xml`
- Create: `src/a2w_fastlio2_bringup/launch/jt128_mapping.launch.py`
- Create: `src/a2w_fastlio2_bringup/config/jt128.yaml`
- Create: `config/cyclonedds_unitree_a2.xml.in`
- Create: `scripts/generate_cyclonedds_config.py`
- Create: `scripts/lib/a2w_common.sh`
- Modify: `scripts/run_fastlio_jt128_pc.sh`
- Modify: `scripts/run_fastlio_jt128_mapping.sh`
- Remove: `config/cyclonedds_unitree_a2.xml`
- Remove: `config/jt128_mapping_save.yaml`

**Interfaces:**
- `generate_cyclonedds_config.py --template PATH --output PATH --pc-address IPv4 --peer-address IPv4` writes one validated XML atomically.
- `a2w_resolve_network PEER` exports `A2W_NETWORK_INTERFACE` and `A2W_PC_IP`, honoring explicit values before route detection.
- `run_fastlio_jt128_pc.sh` launches save-disabled mapping and forwards ROS launch arguments.
- `run_fastlio_jt128_mapping.sh` launches save-enabled mapping and defaults `map_file` below repository `maps/`.

- [ ] **Step 1: Implement the minimal validated XML generator**

Use `argparse`, `ipaddress.ip_address`, `xml.etree.ElementTree`, and `os.replace`. Reject non-IPv4 input, replace `@PC_ADDRESS@` and `@PEER_ADDRESS@`, parse the rendered XML before the atomic write, and create only the output parent directory.

- [ ] **Step 2: Run the XML test and verify GREEN**

Run `python3 -m unittest tests/test_runtime_config.py -v`.

Expected: all generator cases pass.

- [ ] **Step 3: Add the bringup package and launch contract**

The launch file loads the package JT128 YAML and invokes `fast_lio/fastlio_mapping`. It declares `use_sim_time`, `rviz`, `rviz_cfg`, `config_file`, `save_map`, and `map_file`; it passes `pcd_save.pcd_save_en` as a typed Boolean and `map_file_path` as a String override. The CMake package installs `launch/` and `config/`.

- [ ] **Step 4: Implement common root/network setup and both launchers**

Resolve the root from `${BASH_SOURCE[0]}`, source overrideable ROS and workspace setup files, prefer an interface directly connected to the peer subnet before falling back to `ip -4 route get`, generate ignored runtime XML, export CycloneDDS variables, and execute the bringup launch. The save wrapper creates only its selected map parent directory.

- [ ] **Step 5: Run launcher tests and verify GREEN**

Run `bash tests/test_portable_launchers.sh` from the repository root and again with the current directory set to `/tmp`.

Expected: both runs pass and report workspace-rooted runtime paths.

### Task 4: Add Bootstrap, Build, and Read-Only Diagnostics

**Files:**
- Create: `scripts/bootstrap.sh`
- Create: `scripts/build.sh`
- Create: `scripts/check_lidar_topics.sh`
- Modify: `tests/test_portable_launchers.sh`

**Interfaces:**
- `bootstrap.sh` initializes recursive submodules and runs rosdep against `src`.
- `build.sh` initializes submodules, sources ROS Humble, and runs `colcon build --symlink-install`.
- `check_lidar_topics.sh` applies the same generated DDS environment and performs only ROS graph/topic reads.

- [ ] **Step 1: Extend the failing launcher test for setup and diagnostic dry-runs**

Inject fake `git`, `rosdep`, `colcon`, and `ros2` binaries through explicit `*_BIN` variables. Assert exact argument capture, root-relative source paths, and absence of `ssh`, service-stop, or robot mutation commands.

- [ ] **Step 2: Run the extended test and verify RED**

Run `bash tests/test_portable_launchers.sh`.

Expected: FAIL because the three scripts do not exist.

- [ ] **Step 3: Implement minimal setup, build, and topic-check scripts**

Each script derives the repository root independently or sources `scripts/lib/a2w_common.sh`, validates required setup files, prints the operation it is about to run, and uses `exec` where no cleanup is required. The topic checker runs `ros2 topic info`, `ros2 topic hz`, or `ros2 topic echo --once` only.

- [ ] **Step 4: Run the extended test and verify GREEN**

Run `bash tests/test_portable_launchers.sh`.

Expected: all script contracts pass.

### Task 5: Repository Hygiene and User Documentation

**Files:**
- Create: `.gitignore`
- Create: `README.md`
- Create: `maps/.gitkeep`
- Modify: `docs/superpowers/plans/2026-09-02-hesai-jt128-fast-lio2-pc.md`
- Modify: `docs/superpowers/specs/2026-09-02-hesai-jt128-fast-lio2-pc-design.md`
- Include: `docs/superpowers/specs/2026-09-03-a2w-fastlio2-repository-design.md`
- Include: `docs/superpowers/plans/2026-09-03-a2w-fastlio2-repository.md`

**Interfaces:**
- README commands are the public install/build/run/save contract.
- `.gitignore` protects all generated paths while keeping `maps/.gitkeep`.

- [ ] **Step 1: Add ignore rules and rewrite historical docs portably**

Ignore `/build/`, `/install/`, `/log/`, `/maps/*` with `!/maps/.gitkeep`, personal agent state, editor state, caches, bags, PCD files, and runtime XML/YAML. Replace fixed local paths in older docs with `$A2W_FASTLIO_ROOT` or repository-relative paths.

- [ ] **Step 2: Write the Chinese README**

Document prerequisites, recursive clone, bootstrap, build, DDS auto-detection and overrides, live mapping, map saving on graceful shutdown, read-only topic diagnostics, directory layout, no-loop-closure limitation, and the explicit robot-side non-modification boundary.

- [ ] **Step 3: Run hygiene test and verify GREEN**

Run `bash tests/test_repository_hygiene.sh` and `bash tests/run_tests.sh`.

Expected: no fixed user paths, all behavior tests pass, and generated/map files remain ignored.

### Task 6: Build, Clone Verification, Final Commit, and Push

**Files:**
- Modify only files required by a reproducible build failure, always with a failing regression test first.

**Interfaces:**
- Produces: tested engineering commit on `main` and matching GitHub `origin/main`.

- [ ] **Step 1: Build from the project script**

Run `scripts/build.sh` and then source `install/setup.bash`.

Expected: both `fast_lio` and `a2w_fastlio2_bringup` build successfully.

- [ ] **Step 2: Verify ROS launch metadata**

Run:

```bash
ros2 launch a2w_fastlio2_bringup jt128_mapping.launch.py --show-args
```

Expected: the six documented launch arguments are present.

- [ ] **Step 3: Commit the engineering version**

Inspect `git status`, staged diff, submodule diff, ignored files, secrets patterns, and files over 50 MB. Commit with:

```bash
git commit -m "feat: make A2W JT128 FAST-LIO2 workspace portable"
```

- [ ] **Step 4: Verify a clean recursive clone**

Clone the local repository into a temporary directory with `--recurse-submodules`, run the test suite there, and build there with isolated `build`, `install`, and `log` bases.

Expected: tests and build pass without referencing the original workspace path.

- [ ] **Step 5: Push without rewriting history**

Fetch the empty/private remote, verify it has no conflicting `main`, then push using the existing dedicated GitHub identity over SSH 443:

```bash
git push -u origin main
```

- [ ] **Step 6: Confirm remote state**

Run `git ls-remote origin refs/heads/main` and compare the returned object ID with local `git rev-parse HEAD`.

Expected: local and remote commit IDs are identical, and the remote history contains the backup commit followed by the engineering commit.
