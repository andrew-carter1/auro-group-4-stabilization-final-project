# ROS2 Workspace Setup

## Clone the Repository

```bash
git clone <repo_url>
cd <repo_name>
```

---

## Build the Workspace

Make sure you have ROS2 installed and sourced:

```bash
source /opt/ros/humble/setup.bash
```

Then build:

```bash
colcon build
```

---

## Source the Workspace

After building:

```bash
source install/setup.bash
```

You need to do this **every new terminal** (or add to `.bashrc`).

---

## Install Dependencies

Run:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

---

## Workflow

* Create a branch for your work:

  ```bash
  git checkout -b <your-branch-name>
  ```
* Commit and push changes:

  ```bash
  git add .
  git commit -m "your message"
  git push origin <your-branch-name>
  ```

---

## Notes

* Do **not** commit `build/`, `install/`, or `log/` (gitignore should take care of this though)
* Always rebuild after pulling new changes:

  ```bash
  colcon build
  ```
* Re-source after building

---

