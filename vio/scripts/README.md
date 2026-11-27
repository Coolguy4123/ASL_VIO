# scripts

Run all of them from the project folder (`ASL_VIO`). Each file starts with a comment that explains how to use it.

| Script | What it does |
|---|---|
| `find_stops.py` | Lists the times the vehicle stood still: good places to start a run |
| `make_bag.py` | CSV logs + images -> ROS 2 bag (fixes image time, removes lens distortion) |
| `run_openvins.sh` | Plays a bag through OpenVINS, saves the results, **then runs the GNSS comparison automatically** (`plot_vs_gnss.py`, `make_foxglove_bag.py`) |
| `plot_vs_gnss.py` | Picture (`vs_gnss.png`) and numbers (`metrics.txt`): OpenVINS path vs GNSS, and the error over time |
| `make_foxglove_bag.py` | Result bag + ground-truth path, to view in Foxglove |
| `make_ground_truth.py` | GNSS -> clean ground-truth csv (made automatically if missing) |
| `check_run.py` | Compares OpenVINS speed with the logged speed (should be 1.0) |
| `rescale_path.py` | Fixes the scale of a path using the logged speed (not pure VIO) |
