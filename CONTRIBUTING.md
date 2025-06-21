# Contributing to Pandect

Thank you for considering contributing to **Pandect**, our ROS2 and event-based vision toolkit! This document outlines how to set up your environment, follow conventions, and submit contributions.

---

## Prerequisites

Before you begin, ensure you have:

- Ubuntu 22.04 (for ROS 2 Humble)
- Git, Python, and `pip` installed
- ROS 2 Humble installed
- `pre-commit` installed and configured

## Branching strategy
We use a simple but strict Git flow:

- `main`: always working and tested
- `dev`: active development
- `feature/<yourname>-<feature>`: new features or experiments

Always branch from dev, not main.

# Build and test
To build the workspace:
```
cd <your_workspace>
source /opt/ros/${ROS_DISTRO}/setup.bash
colcon build --event-handlers console_cohesion+
source install/setup.bash
```

To run pre-commit hooks:
```
pre-commit run --all-files
```

Make sure your code builds and passes all formatting checks before committing.
