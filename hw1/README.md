## Usage
`python forward_kinematics_qt2126.py`

## Implementation
1. Find homogeneous transformation for each data point: form eight homogeneous transformations (`form_homo_trans`) from eight sets of DH parameters and multiply them (`fk`).
2. Multiply the result from 1. with (0, 0, 0, 1)^T (`convert_frame`). This will convert the position of the marker tip (origin of the tool frame) to coordinates in the O_0 frame.
3. Plot the 3D drawing (`plot_drawing`).
