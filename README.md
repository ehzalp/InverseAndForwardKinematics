A project for implementing inverse and forward kinematics algorithms.
## Overview

A Python 3.12 implementation of forward and inverse kinematics for a planar 3-link robotic arm (3R robot). This project includes workspace analysis, Jacobian computation, and singularity detection.

## Features

- **Forward Kinematics**: Denavit-Hartenberg (DH) parameter-based position and orientation computation
- **Inverse Kinematics**: Analytical solutions for planar 3R configuration
- **Workspace Generation**: Reachable workspace visualization with theoretical boundaries
- **Jacobian Analysis**: Singularity detection and workspace characterization
- **Configuration Management**: Load robot parameters from text files

## Project Structure

- `ForwardKinematic.py`: DH transforms and forward kinematics
- `InverseKinematic.py`: Analytical IK solutions and testing
- `workspace.py`: Workspace point generation and visualization
- `config.py`: Robot parameter loading and configuration
- `jacobian.py`: Jacobian computation and singularity analysis

## Usage

```bash
python ForwardKinematic.py      # Test forward kinematics
python InverseKinematic.py      # Test inverse kinematics
python workspace.py              # Generate workspace visualization
```

## Requirements

- Python 3.12
- NumPy
- Matplotlib
