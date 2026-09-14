Replaced blocking cv2.selectROIs with same-window sequential selection, mouse confirmation buttons and Enter support. Last confirmation starts inference; repeated confirmation without a new rectangle cannot duplicate ROIs.

Validation: 9 pytest tests passed; py_compile and git diff --check passed. Installed preview is a symlink to edited source; no rebuild needed. Existing running processes need a viewer relaunch. No camera, controller, or robot started/stopped for this change. Desktop mouse interaction still requires live user validation.
