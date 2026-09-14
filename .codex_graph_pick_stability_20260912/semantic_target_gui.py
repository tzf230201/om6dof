#!/usr/bin/env python3
"""Desktop-only semantic target selector for DD-GNG planning previews."""

import json
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


COCO_CLASSES = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat",
    "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog",
    "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
    "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite",
    "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle",
    "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich",
    "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
    "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote",
    "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "book",
    "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush",
]


class SemanticTargetGui(Node):
    def __init__(self):
        super().__init__("semantic_target_gui")
        self.publisher = self.create_publisher(
            String, "/om6dof_topo_gng_v2/set_target_classes", 1)
        self.create_subscription(
            String, "/om6dof_topo_gng_v2/object_clusters", self._clusters, 2)
        self.plan_client = self.create_client(Trigger, "/plan_graph_pick")
        self.execute_client = self.create_client(Trigger, "/execute_graph_pick")
        self.visible_classes = set()
        self._lock = threading.Lock()

    def _clusters(self, message):
        try:
            clusters = json.loads(message.data)
            classes = {entry["class"] for entry in clusters if entry.get("class") in COCO_CLASSES}
        except (json.JSONDecodeError, TypeError, KeyError):
            return
        with self._lock:
            self.visible_classes = classes

    def choose(self, value):
        target = "all" if value == "All detected classes" else value
        message = String()
        message.data = target
        self.publisher.publish(message)
        self.status.set("Target planning: all classes" if target == "all" else f"Target planning: {target}")

    def preview_path(self):
        if not self.plan_client.wait_for_service(timeout_sec=0.2):
            self.status.set("Planning service is starting; try again shortly")
            return
        self.status.set("Computing EoE-to-object preview...")
        future = self.plan_client.call_async(Trigger.Request())

        def completed(done):
            try:
                response = done.result()
                text = response.message
            except Exception as error:  # ROS transport error is surfaced in the GUI only.
                text = f"Planning request failed: {error}"
            self.get_logger().info(text)
            self._plan_status = text

        future.add_done_callback(completed)

    def execute_path(self):
        if not self.execute_client.wait_for_service(timeout_sec=0.2):
            self.status.set("Execution service is starting; try again shortly")
            return
        self.status.set("Checking execution interlocks...")
        future = self.execute_client.call_async(Trigger.Request())

        def completed(done):
            try:
                response = done.result()
                text = response.message
            except Exception as error:
                text = f"Execution request failed: {error}"
            self.get_logger().info(text)
            self._plan_status = text

        future.add_done_callback(completed)

    def run(self):
        import tkinter as tk
        from tkinter import ttk

        root = tk.Tk()
        root.title("DD-GNG Planning Target")
        root.geometry("430x270")
        root.resizable(False, False)
        ttk.Label(root, text="Object target for EoE planning preview", font=("Sans", 13, "bold")).pack(pady=(20, 8))
        ttk.Label(root, text="Preview first. Execute requires an explicit button click.").pack()
        selected = tk.StringVar(value="All detected classes")
        menu = ttk.Combobox(root, textvariable=selected, state="readonly", width=34)
        menu["values"] = ["All detected classes"] + COCO_CLASSES
        menu.pack(pady=15)
        self.status = tk.StringVar(value="Waiting for valid DD-GNG object clusters")
        ttk.Label(root, textvariable=self.status).pack(pady=4)
        ttk.Button(root, text="Set planning target", command=lambda: self.choose(selected.get())).pack(pady=8)
        ttk.Button(root, text="Preview EoE path in RViz", command=self.preview_path).pack()
        ttk.Button(root, text="Execute planned motion", command=self.execute_path).pack(pady=8)
        self._plan_status = ""

        def refresh_visible():
            with self._lock:
                visible = sorted(self.visible_classes)
            if visible:
                menu["values"] = ["All detected classes"] + visible + [
                    item for item in COCO_CLASSES if item not in visible]
            if self._plan_status:
                self.status.set(self._plan_status)
                self._plan_status = ""
            root.after(500, refresh_visible)

        root.protocol("WM_DELETE_WINDOW", lambda: (root.destroy(), rclpy.shutdown()))
        refresh_visible()
        root.mainloop()


def main():
    rclpy.init()
    node = SemanticTargetGui()
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spinner.join(timeout=1)


if __name__ == "__main__":
    main()
