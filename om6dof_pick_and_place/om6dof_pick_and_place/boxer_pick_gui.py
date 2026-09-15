"""Small non-blocking desktop UI for the independent Boxer centre picker."""
import json
import os
import threading
import time
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger


class BoxerPickGUI(Node):
    def __init__(self):
        super().__init__('boxer_pick_gui')
        self.lock = threading.Lock()
        self.payload, self.received, self.notice = {}, 0.0, ''
        self.pending = None
        # rclpy.node.Node already exposes a read-only `clients` property
        # (its own service-client list), so this must use a different name.
        self.trigger_clients = {name: self.create_client(Trigger, '/boxer_pick/' + name)
                                for name in ('preview', 'execute', 'cancel')}
        self.create_subscription(String, '/boxer_pick/status', self.status,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def status(self, message):
        try:
            value = json.loads(message.data)
        except ValueError:
            return
        with self.lock:
            self.payload, self.received = value, time.monotonic()

    def call(self, name):
        client = self.trigger_clients[name]
        if not client.service_is_ready():
            with self.lock:
                self.notice = 'Node pickup belum siap'
            return
        if self.pending is not None and not self.pending.done() and name != 'cancel':
            return
        future = client.call_async(Trigger.Request())
        self.pending = future
        def done(reply):
            try:
                notice = reply.result().message
            except Exception as error:
                notice = str(error)
            with self.lock:
                self.notice = notice
        future.add_done_callback(done)


def main(args=None):
    # This is an optional convenience window on top of boxer_pick_node's own
    # service interface; a headless launch (over SSH, no X forwarding) used
    # to crash here with a raw Tcl traceback and exit code 1 instead of just
    # skipping the GUI, which under some launch files brings the rest of the
    # pick pipeline down with it via shutdown-on-process-exit.
    if not os.environ.get('DISPLAY'):
        print('boxer_pick_gui: no DISPLAY set; skipping the desktop UI '
              '(use the /boxer_pick/preview, execute, cancel services directly)',
              flush=True)
        return
    rclpy.init(args=args)
    node = BoxerPickGUI()
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()
    root = tk.Tk()
    root.title('YOLO → Boxer3D → Pickup tengah')
    root.geometry('670x350')
    ttk.Label(root, text='Pickup pusat kotak Boxer3D', font=('', 18, 'bold')).pack(pady=15)
    target, status, notice = tk.StringVar(), tk.StringVar(), tk.StringVar()
    ttk.Label(root, textvariable=target).pack(pady=5)
    ttk.Label(root, textvariable=status, wraplength=620, justify='center').pack(pady=12)
    buttons = ttk.Frame(root)
    buttons.pack(pady=12)
    preview = ttk.Button(buttons, text='1. Preview', command=lambda: node.call('preview'))
    preview.pack(side='left', padx=8)
    execute = ttk.Button(buttons, text='2. Dekati dan jepit', command=lambda: node.call('execute'))
    execute.pack(side='left', padx=8)
    ttk.Button(buttons, text='Stop', command=lambda: node.call('cancel')).pack(side='left', padx=8)
    ttk.Label(root, textvariable=notice, wraplength=620).pack(pady=8)
    ttk.Label(root, text='Titik orange = pusat kotak. Periksa lintasan di RViz sebelum pickup.').pack(pady=5)
    def update():
        with node.lock:
            payload, received, message = dict(node.payload), node.received, node.notice
        fresh = time.monotonic() - received < 2.0
        target.set('Target: ' + str(payload.get('target_class', 'bottle')))
        status.set(str(payload.get('message', 'Menunggu node pickup')) if fresh else 'Status pickup belum tersedia')
        notice.set(message)
        preview.state(['disabled'] if not fresh or payload.get('busy') else ['!disabled'])
        execute.state(['!disabled'] if fresh and payload.get('executable') else ['disabled'])
        root.after(150, update)
    update()
    try:
        root.mainloop()
    finally:
        # Closing the window requests cancellation of this picker's goals only.
        node.call('cancel')
        time.sleep(.1)
        if rclpy.ok():
            rclpy.shutdown()
        spinner.join(timeout=2)
        node.destroy_node()
