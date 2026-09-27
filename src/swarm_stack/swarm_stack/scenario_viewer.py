"""Readable animated mission map using Python's standard Tk canvas.

--live subscribes to ROS telemetry. The default is explicitly labeled as the
kinematic reference model and does not command ArduPilot or Gazebo.
"""
import argparse
import math
import time
import tkinter as tk
from types import SimpleNamespace

from .scenario import Scenario
from .reference_sim import ReferenceSwarm

COLORS = {'relay':'#f4c76b','surveyor':'#58d5eb','returning':'#c995ff','rtl':'#c995ff',
          'landed':'#8293ad','idle':'#8293ad','charging':'#54d39b','failed':'#ff7083'}
BG='#101a2b'; PANEL='#18253a'; TEXT='#eaf1fb'; MUTED='#9aaec9'


class LiveSwarm:
    def __init__(self, scenario):
        import rclpy
        from swarm_interfaces.msg import UAVStatus, PoI, ConnectivityGraph, CommunicationStats
        from rosgraph_msgs.msg import Clock
        self.rclpy=rclpy; rclpy.init()
        self.node=rclpy.create_node('swarm_dashboard')
        self.scenario=scenario; self.time=0.0; self.states={}; self.pois=[]; self.events=[]
        self.wall_start=time.monotonic(); self.clock_wall_start=None; self.clock_at=None; self.clock_epoch=None
        self.graph_msg=None; self.comms=None; self.minimum_separation=float('inf')
        self.node.create_subscription(UAVStatus,'/swarm/uav_status',self.status,10)
        self.node.create_subscription(PoI,'/swarm/poi_updates',self.poi,10)
        self.node.create_subscription(ConnectivityGraph,'/swarm/connectivity_graph',lambda m:setattr(self,'graph_msg',m),10)
        self.node.create_subscription(CommunicationStats,'/swarm/communication_stats',lambda m:setattr(self,'comms',m),10)
        self.node.create_subscription(Clock,'/drone1/clock',self.clock,10)
        self.events.append((0,'Waiting for live Gazebo telemetry on ROS_DOMAIN_ID...'))

    def clock(self, msg):
        now=msg.clock.sec+msg.clock.nanosec/1e9
        if self.clock_epoch is None:
            self.clock_epoch=now
            self.clock_wall_start=time.monotonic()
        self.time=max(0.0,now-self.clock_epoch)
        self.clock_at=time.monotonic()

    @property
    def wall_elapsed(self):
        return time.monotonic()-self.wall_start

    @property
    def realtime_factor(self):
        if self.clock_wall_start is None:
            return 0.0
        return self.time/max(1.0,time.monotonic()-self.clock_wall_start)

    @property
    def drones(self):
        return sorted(self.states.values(),key=lambda d:int(d.name.replace('drone','')))

    def status(self,m):
        p=(m.position.x,m.position.y,m.position.z)
        if not m.position_valid:return
        old=self.states.get(m.name)
        trail=old.trail if old else []
        if not trail or math.dist(trail[-1],p)>2:trail=(trail+[p])[-250:]
        if old is None or old.role != m.role:self.events.append((self.time,f'{m.name}: {m.flight_state} / {m.role}'))
        self.states[m.name]=SimpleNamespace(name=m.name,position=p,role=m.role,trail=trail,
            flight_s=m.flight_elapsed_s,battery=m.battery_pct,flight_state=m.flight_state,
            task_id=m.current_task_id)

    def poi(self,m):
        old=next((p for p in self.pois if p['id']==m.id),None)
        if old is None:
            old=dict(id=m.id,spawn_s=self.time,reported_s=None);self.pois.append(old)
            self.events.append((self.time,f'New PoI: {m.id}'))
        if m.surveyed and old['reported_s'] is None:self.events.append((self.time,f'GCS received {m.id}'))
        old.update(x=m.position.x,y=m.position.y,priority=m.priority,reported_s=self.time if m.surveyed else None)

    def graph(self):
        names=['GCS']+[d.name for d in self.drones];edges=[];adj={i:[] for i in range(len(names))}
        if self.graph_msg:
            for e in self.graph_msg.edges:
                if e.link_up and e.node_a in names and e.node_b in names:
                    a,b=names.index(e.node_a),names.index(e.node_b);edges.append((a,b));adj[a].append(b);adj[b].append(a)
        hops={0:0};parents={};q=[0]
        for a in q:
            for b in adj[a]:
                if b not in hops:hops[b]=hops[a]+1;parents[b]=a;q.append(b)
        return hops,parents,edges

    def summary(self):
        for i,a in enumerate(self.drones):
            for b in self.drones[i+1:]:self.minimum_separation=min(self.minimum_separation,math.dist(a.position,b.position))
        return dict(reported=sum(p['reported_s'] is not None for p in self.pois),poi_count=self.scenario.poi_count,
                    min_separation_m=self.minimum_separation,relay_reallocations=0,
                    pdr_pct=100*self.comms.packets_delivered/max(1,self.comms.packets_sent) if self.comms else 0)

    def poll(self):
        for _ in range(12):self.rclpy.spin_once(self.node,timeout_sec=0)

    def close(self):
        self.node.destroy_node();self.rclpy.shutdown()


class Dashboard:
    def __init__(self, source, live=False):
        self.source, self.live = source, live
        self.root = tk.Tk()
        self.root.title('UAV-X | ' + ('LIVE Gazebo telemetry' if live else 'Kinematic sample simulation'))
        self.root.geometry('1440x920')
        self.root.minsize(1000, 690)
        self.root.configure(bg=BG)
        self.running, self.speed, self.accumulator = True, 1.0, 0.0
        self.last = time.monotonic()
        self.closed, self.after_id = False, None
        self.zoom, self.pan, self.drag = 1.0, [0.0, 0.0], None
        self.selected, self.selected_poi = None, None
        self.follow, self.show_ranges, self.show_trails, self.show_links = False, False, True, True
        self.poi_filter = 'All'
        self.tab = 'Fleet'
        self._fleet_rows, self._poi_rows = [], []
        self._build()
        self.root.protocol('WM_DELETE_WINDOW', self.close)
        self.tick()

    def _label(self, parent, text, size=10, fg=TEXT, bold=False, **kwargs):
        return tk.Label(parent, text=text, bg=parent['bg'], fg=fg,
                        font=('DejaVu Sans', size, 'bold' if bold else 'normal'), **kwargs)

    def _button(self, parent, text, command, color='#263c57'):
        button = tk.Button(parent, text=text, command=command, bg=color, fg=TEXT,
                           activebackground='#365776', activeforeground=TEXT,
                           relief='flat', padx=9, pady=5, font=('DejaVu Sans', 10),
                           cursor='hand2')
        button.pack(side='left', padx=(0, 6))
        return button

    def _build(self):
        header = tk.Frame(self.root, bg=BG, padx=18, pady=12)
        header.pack(fill='x')
        self._label(header, 'UAV-X   /   SWARM OPERATIONS', 18, bold=True).pack(side='left')
        self.mode_label = self._label(header, '● LIVE GAZEBO' if self.live else '● KINEMATIC MODEL',
                                      11, '#58d5eb', True)
        self.mode_label.pack(side='right')

        toolbar = tk.Frame(self.root, bg=PANEL, padx=12, pady=8)
        toolbar.pack(fill='x', padx=16)
        transport = tk.Frame(toolbar, bg=PANEL)
        transport.pack(side='left')
        self.pause_button = self._button(transport, 'Pause', self.pause)
        if self.live:
            self.pause_button.config(state='disabled')
        self.speed_buttons = {}
        for speed in (1, 10, 30, 60):
            button = self._button(transport, f'{speed}×', lambda value=speed: self.set_speed(value))
            if self.live:
                button.config(state='disabled')
            self.speed_buttons[speed] = button
        self._button(toolbar, 'Fit area', self.fit)
        self.follow_button = self._button(toolbar, 'Follow: off', self.toggle_follow)
        self.range_button = self._button(toolbar, 'Radio ranges', self.toggle_ranges)
        self.trail_button = self._button(toolbar, 'Trails: on', self.toggle_trails)
        self.link_button = self._button(toolbar, 'Links: on', self.toggle_links)
        self.clock_label = self._label(toolbar, '', 11, bold=True)
        self.clock_label.pack(side='right', padx=5)
        self._update_buttons()

        if not self.live:
            actions = tk.Frame(self.root, bg=BG, padx=18, pady=7)
            actions.pack(fill='x')
            self._label(actions, 'SCENARIO  ', 9, MUTED, True).pack(side='left')
            self._button(actions, '+ Urgent PoI', self.source.add_emergency_poi, '#36516b')
            self._button(actions, 'Fail relay', self.source.fail_relay, '#5c3d4f')
            self._button(actions, 'Degrade link', self.source.degrade_relay, '#5c4938')
            self._button(actions, '+1 min', lambda: self.jump(60))
            self._button(actions, '+5 min', lambda: self.jump(300))
            self._label(actions, 'Events change the reference model only', 9, MUTED).pack(side='right')

        body = tk.Frame(self.root, bg=BG)
        body.pack(fill='both', expand=True, padx=16, pady=10)
        map_panel = tk.Frame(body, bg='#111f31')
        map_panel.pack(side='left', fill='both', expand=True)
        self.canvas = tk.Canvas(map_panel, bg='#111f31', highlightthickness=0, cursor='crosshair')
        self.canvas.pack(fill='both', expand=True)
        self.map_hint = self._label(map_panel, 'Scroll: zoom  •  Drag: pan  •  Click: inspect  •  F: fit  •  Esc: clear',
                                    9, MUTED, anchor='w', padx=10, pady=5)
        self.map_hint.pack(fill='x')

        side = tk.Frame(body, bg=PANEL, width=320, padx=12, pady=10)
        side.pack(side='right', fill='y', padx=(10, 0))
        side.pack_propagate(False)
        self._label(side, 'MISSION STATUS', 13, bold=True).pack(anchor='w')
        self.stats = self._label(side, '', 10, anchor='w', justify='left')
        self.stats.pack(fill='x', pady=(8, 4))
        self.progress = tk.Canvas(side, height=13, bg='#25374d', highlightthickness=0)
        self.progress.pack(fill='x', pady=(2, 6))
        self.warning = self._label(side, '', 9, '#f4c76b', justify='left', wraplength=285, anchor='w')
        self.warning.pack(fill='x', pady=(0, 9))

        tabs = tk.Frame(side, bg=PANEL)
        tabs.pack(fill='x')
        self.fleet_tab = self._button(tabs, 'Aircraft', lambda: self.show_tab('Fleet'))
        self.poi_tab = self._button(tabs, 'PoIs', lambda: self.show_tab('PoIs'))
        self.list_area = tk.Frame(side, bg=PANEL)
        self.list_area.pack(fill='both', expand=True, pady=(7, 5))
        self.fleet_list = tk.Listbox(self.list_area, bg='#142136', fg=TEXT, selectbackground='#316078',
                                     selectforeground=TEXT, activestyle='none', borderwidth=0,
                                     highlightthickness=0, font=('DejaVu Sans Mono', 10), exportselection=False)
        self.poi_list = tk.Listbox(self.list_area, bg='#142136', fg=TEXT, selectbackground='#316078',
                                   selectforeground=TEXT, activestyle='none', borderwidth=0,
                                   highlightthickness=0, font=('DejaVu Sans Mono', 10), exportselection=False)
        self.fleet_list.bind('<<ListboxSelect>>', self.select_fleet_row)
        self.poi_list.bind('<<ListboxSelect>>', self.select_poi_row)
        self.filter_row = tk.Frame(side, bg=PANEL)
        for label in ('All', 'Pending', 'Reported'):
            self._button(self.filter_row, label, lambda value=label: self.set_poi_filter(value))
        self.show_tab('Fleet')
        self._label(side, 'SELECTED', 9, MUTED, True).pack(anchor='w', pady=(4, 2))
        self.detail = self._label(side, 'Click an aircraft or PoI to inspect it.', 10,
                                  justify='left', anchor='nw', wraplength=285)
        self.detail.pack(fill='x', pady=(0, 8))
        detail_actions = tk.Frame(side, bg=PANEL)
        detail_actions.pack(fill='x')
        self._button(detail_actions, 'Center selection', self.center_selection)

        footer = tk.Frame(self.root, bg=PANEL, padx=14, pady=8)
        footer.pack(fill='x', padx=16, pady=(0, 12))
        self._label(footer, 'SURVEYOR cyan   RELAY amber   RETURN violet   PoI pending red / reported green',
                    9, MUTED, anchor='w').pack(fill='x')
        self.event_label = self._label(footer, '', 9, justify='left', anchor='w')
        self.event_label.pack(fill='x', pady=(5, 0))

        self.canvas.bind('<ButtonPress-1>', self.down)
        self.canvas.bind('<B1-Motion>', self.motion)
        self.canvas.bind('<ButtonRelease-1>', self.release)
        self.canvas.bind('<Motion>', self.hover)
        self.canvas.bind('<Leave>', lambda event: self.map_hint.config(
            text='Scroll: zoom  •  Drag: pan  •  Click: inspect  •  F: fit  •  Esc: clear'))
        self.canvas.bind('<MouseWheel>', self.wheel)
        self.canvas.bind('<Button-4>', lambda event: self.wheel(event, 1.15))
        self.canvas.bind('<Button-5>', lambda event: self.wheel(event, 1 / 1.15))
        self.root.bind('<Key-f>', lambda event: self.fit())
        self.root.bind('<Escape>', lambda event: self.clear_selection())
        self.root.bind('<space>', lambda event: self.pause() if not self.live else None)
        self.root.bind('<plus>', lambda event: self.zoom_by(1.15))
        self.root.bind('<minus>', lambda event: self.zoom_by(1 / 1.15))

    def _update_buttons(self):
        for speed, button in self.speed_buttons.items():
            button.config(bg='#38718a' if not self.live and self.speed == speed else '#263c57')
        self.follow_button.config(text='Follow: on' if self.follow else 'Follow: off',
                                  bg='#38718a' if self.follow else '#263c57')
        self.range_button.config(bg='#38718a' if self.show_ranges else '#263c57')
        self.trail_button.config(text='Trails: on' if self.show_trails else 'Trails: off',
                                 bg='#38718a' if self.show_trails else '#263c57')
        self.link_button.config(text='Links: on' if self.show_links else 'Links: off',
                                bg='#38718a' if self.show_links else '#263c57')

    def pause(self):
        if self.live:
            return
        self.running = not self.running
        self.pause_button.config(text='Pause' if self.running else 'Resume',
                                 bg='#263c57' if self.running else '#38718a')

    def set_speed(self, value):
        if not self.live:
            self.speed = float(value)
            self._update_buttons()

    def jump(self, seconds):
        if self.live:
            return
        target = min(self.source.scenario.mission_duration_s, self.source.time + seconds)
        self.accumulator = 0.0
        self.root.config(cursor='watch')
        self.root.update_idletasks()
        try:
            while self.source.time < target:
                self.source.step(min(1.0, target - self.source.time))
        finally:
            self.root.config(cursor='')
            self.last = time.monotonic()
        self.draw()

    def fit(self):
        self.zoom, self.pan, self.follow = 1.0, [0.0, 0.0], False
        self._update_buttons()
        self.draw()

    def toggle_follow(self):
        if self.selected is None and self.selected_poi is None:
            self.map_hint.config(text='Select an aircraft or PoI first, then turn on Follow.')
            return
        self.follow = not self.follow
        self._update_buttons()
        if self.follow:
            self.center_selection()

    def toggle_ranges(self):
        self.show_ranges = not self.show_ranges
        self._update_buttons()
        self.draw()

    def toggle_trails(self):
        self.show_trails = not self.show_trails
        self._update_buttons()
        self.draw()

    def toggle_links(self):
        self.show_links = not self.show_links
        self._update_buttons()
        self.draw()

    def show_tab(self, tab):
        self.tab = tab
        self.fleet_list.pack_forget()
        self.poi_list.pack_forget()
        self.filter_row.pack_forget()
        if tab == 'Fleet':
            self.fleet_list.pack(fill='both', expand=True)
        else:
            self.poi_list.pack(fill='both', expand=True)
            self.filter_row.pack(fill='x', pady=(4, 0))
        self.fleet_tab.config(bg='#38718a' if tab == 'Fleet' else '#263c57')
        self.poi_tab.config(bg='#38718a' if tab == 'PoIs' else '#263c57')

    def set_poi_filter(self, value):
        self.poi_filter = value
        self.draw()

    def select_fleet_row(self, event=None):
        indices = self.fleet_list.curselection()
        if indices and indices[0] < len(self._fleet_rows):
            self.selected, self.selected_poi = self._fleet_rows[indices[0]], None
            if self.follow:
                self.center_selection()
            else:
                self.draw()

    def select_poi_row(self, event=None):
        indices = self.poi_list.curselection()
        if indices and indices[0] < len(self._poi_rows):
            self.selected_poi, self.selected = self._poi_rows[indices[0]], None
            if self.follow:
                self.center_selection()
            else:
                self.draw()

    def clear_selection(self):
        self.selected = self.selected_poi = None
        self.follow = False
        self._update_buttons()
        self.draw()

    def selection_position(self):
        if self.selected:
            drone = next((d for d in self.source.drones if d.name == self.selected), None)
            return drone.position if drone else None
        if self.selected_poi:
            poi = next((p for p in self.source.pois if p['id'] == self.selected_poi), None)
            return (poi['x'], poi['y']) if poi else None
        return None

    def center_selection(self):
        point = self.selection_position()
        if point is None:
            return
        x, y = self.xy(point)
        self.pan[0] += self.canvas.winfo_width() / 2 - x
        self.pan[1] += self.canvas.winfo_height() / 2 - y
        self.draw()

    def down(self, event):
        self.drag = (event.x, event.y)
        for d in reversed(self.source.drones):
            x, y = self.xy(d.position)
            if math.hypot(event.x - x, event.y - y) <= 13:
                self.selected, self.selected_poi = d.name, None
                self.show_tab('Fleet')
                self.draw()
                return
        for p in reversed(self.source.pois):
            if p['spawn_s'] > self.source.time:
                continue
            x, y = self.xy((p['x'], p['y']))
            if math.hypot(event.x - x, event.y - y) <= 13:
                self.selected, self.selected_poi = None, p['id']
                self.show_tab('PoIs')
                self.draw()
                return
        self.clear_selection()

    def motion(self, event):
        if self.drag:
            self.pan[0] += event.x - self.drag[0]
            self.pan[1] += event.y - self.drag[1]
            self.drag = (event.x, event.y)
            self.follow = False
            self._update_buttons()
            self.draw()

    def release(self, event):
        self.drag = None

    def hover(self, event):
        for d in self.source.drones:
            if math.hypot(event.x - self.xy(d.position)[0], event.y - self.xy(d.position)[1]) <= 12:
                self.map_hint.config(text=f'{d.name}  •  {d.role}  •  altitude {d.position[2]:.0f} m  •  click to inspect')
                return
        for p in self.source.pois:
            if p['spawn_s'] <= self.source.time and math.hypot(event.x - self.xy((p['x'], p['y']))[0],
                                                                event.y - self.xy((p['x'], p['y']))[1]) <= 13:
                status = 'reported' if p['reported_s'] is not None else 'pending'
                self.map_hint.config(text=f"{p['id']}  •  priority {p['priority']}  •  {status}  •  click to inspect")
                return
        self.map_hint.config(text='Scroll: zoom  •  Drag: pan  •  Click: inspect  •  F: fit  •  Esc: clear')

    def wheel(self, event, factor=None):
        factor = factor or (1.15 if event.delta > 0 else 1 / 1.15)
        self.zoom_by(factor, (event.x, event.y))

    def zoom_by(self, factor, anchor=None):
        old = self.zoom
        self.zoom = max(0.6, min(8.0, old * factor))
        if anchor is None:
            anchor = (self.canvas.winfo_width() / 2, self.canvas.winfo_height() / 2)
        cx, cy = self.canvas.winfo_width() / 2, self.canvas.winfo_height() / 2
        scale = self.zoom / old
        self.pan = [anchor[0] - cx - (anchor[0] - cx - self.pan[0]) * scale,
                    anchor[1] - cy - (anchor[1] - cy - self.pan[1]) * scale]
        self.draw()

    def xy(self, point):
        width, height = self.canvas.winfo_width(), self.canvas.winfo_height()
        scenario = self.source.scenario
        x0 = min(scenario.base_area[0], scenario.operational_area[0]) - 50
        x1 = max(scenario.base_area[2], scenario.operational_area[2]) + 50
        y0 = min(scenario.base_area[1], scenario.operational_area[1]) - 50
        y1 = max(scenario.base_area[3], scenario.operational_area[3]) + 50
        self.scale = min(max(1, width - 70) / (x1 - x0),
                         max(1, height - 70) / (y1 - y0)) * self.zoom
        return (width / 2 + (point[0] - (x0 + x1) / 2) * self.scale + self.pan[0],
                height / 2 - (point[1] - (y0 + y1) / 2) * self.scale + self.pan[1])

    def _text(self, x, y, value, color=TEXT, size=10, **kwargs):
        self.canvas.create_text(x, y, text=value, fill=color,
                                font=('DejaVu Sans', size), **kwargs)

    def _draw_map(self, drones, parents):
        canvas, scenario = self.canvas, self.source.scenario
        canvas.delete('all')
        x0, y0, x1, y1 = scenario.operational_area
        a, b = self.xy((x0, y1)), self.xy((x1, y0))
        canvas.create_rectangle(*a, *b, fill='#172c40', outline='#5d819d', width=2)
        for x in range(int(x0), int(x1) + 1, 100):
            canvas.create_line(*self.xy((x, y0)), *self.xy((x, y1)), fill='#294055')
        for y in range(int(y0), int(y1) + 1, 100):
            canvas.create_line(*self.xy((x0, y)), *self.xy((x1, y)), fill='#294055')
        self._text((a[0] + b[0]) / 2, a[1] - 20, f'OPERATIONAL AREA   {x1-x0:g} × {y1-y0:g} m')
        base = scenario.base_area
        u, v = self.xy((base[0], base[3])), self.xy((base[2], base[1]))
        canvas.create_rectangle(*u, *v, fill='#173b46', outline='#428591')
        corridor = scenario.corridor_area
        u, v = self.xy((corridor[0], corridor[3])), self.xy((corridor[2], corridor[1]))
        canvas.create_rectangle(*u, *v, outline='#4a7880', dash=(4, 5))
        gcs = self.xy(scenario.gcs_position)
        canvas.create_oval(gcs[0]-8, gcs[1]-8, gcs[0]+8, gcs[1]+8, fill='#75aaff', outline='')
        self._text(gcs[0], gcs[1]-21, 'GCS', '#a9c7ff')
        self._text(a[0]+8, b[1]+19, '100 m grid  •  radio max 100 m  •  altitude max 100 m', MUTED, 9, anchor='w')
        points = [tuple(scenario.gcs_position)] + [d.position for d in drones]
        if self.show_links:
            for child, parent in parents.items():
                canvas.create_line(*self.xy(points[child]), *self.xy(points[parent]),
                                   fill='#397d70', width=2)
        for d in drones:
            color = COLORS.get(d.role, COLORS['idle'])
            if self.show_trails and len(d.trail) > 1:
                canvas.create_line(*[coord for point in d.trail for coord in self.xy(point)],
                                   fill='#4c677e', width=1)
            x, y = self.xy(d.position)
            if self.show_ranges or d.name == self.selected:
                radius = scenario.comm_range_m * self.scale
                canvas.create_oval(x-radius, y-radius, x+radius, y+radius,
                                   outline='#477185', dash=(4, 5))
            if d.name == self.selected:
                canvas.create_oval(x-13, y-13, x+13, y+13, outline=TEXT, width=2)
            canvas.create_line(x-6, y-6, x+6, y+6, fill=color, width=2)
            canvas.create_line(x+6, y-6, x-6, y+6, fill=color, width=2)
            canvas.create_oval(x-4, y-4, x+4, y+4, fill=color, outline=BG)
            self._text(x+10, y-10, d.name.replace('drone', 'D'), color, 9, anchor='w')
        for poi in self.source.pois:
            if poi['spawn_s'] > self.source.time:
                continue
            x, y = self.xy((poi['x'], poi['y']))
            done = poi['reported_s'] is not None
            color = '#58d39a' if done else '#ff7486'
            if poi['id'] == self.selected_poi:
                canvas.create_oval(x-15, y-15, x+15, y+15, outline=TEXT, width=2)
            canvas.create_oval(x-7, y-7, x+7, y+7, fill=color, outline=BG, width=2)
            self._text(x+12, y-3, poi['id'].replace('poi_', 'P') + (' ✓' if done else ''),
                       color, 9, anchor='w')

    def _refresh_lists(self, drones):
        fleet = [d.name for d in drones]
        rows = [p for p in self.source.pois if p['spawn_s'] <= self.source.time and
                (self.poi_filter == 'All' or
                 (self.poi_filter == 'Reported') == (p['reported_s'] is not None))]
        self._fleet_rows = fleet
        self._poi_rows = [p['id'] for p in rows]
        self.fleet_list.delete(0, tk.END)
        for index, drone in enumerate(drones):
            pct = getattr(drone, 'battery', max(0, 100 * (1 - drone.flight_s / self.source.scenario.uav_endurance_s)))
            state = getattr(drone, 'flight_state', drone.role)
            task = getattr(drone, 'task_id', '')
            self.fleet_list.insert(tk.END, f' {drone.name:7} {state[:9]:9} {task[:10]:10} {pct:3.0f}%')
            self.fleet_list.itemconfig(index, fg=COLORS.get(drone.role, TEXT))
        if self.selected in fleet:
            self.fleet_list.selection_set(fleet.index(self.selected))
        self.poi_list.delete(0, tk.END)
        for index, poi in enumerate(rows):
            status = 'reported' if poi['reported_s'] is not None else 'pending'
            self.poi_list.insert(tk.END, f" {poi['id'][:12]:12} P{poi['priority']:2}  {status}")
            self.poi_list.itemconfig(index, fg='#58d39a' if status == 'reported' else '#ff7486')
        if self.selected_poi in self._poi_rows:
            self.poi_list.selection_set(self._poi_rows.index(self.selected_poi))

    def _refresh_detail(self, drones, hops):
        if self.selected:
            drone = next((d for d in drones if d.name == self.selected), None)
            if drone:
                index = drones.index(drone) + 1
                pct = getattr(drone, 'battery', max(0, 100 * (1-drone.flight_s/self.source.scenario.uav_endurance_s)))
                connection = f'{hops[index]} hops' if index in hops else 'disconnected'
                task = getattr(drone, 'task_id', '')
                self.detail.config(text=f'{drone.name}  •  {drone.role}\n'
                                        f'Task: {task or "none"}  |  Battery: {pct:.0f}%\n'
                                        f'State: {getattr(drone, "flight_state", drone.role)}\n'
                                        f'Position: {drone.position[0]:.0f}, {drone.position[1]:.0f} m\n'
                                        f'Altitude: {drone.position[2]:.1f} m  |  GCS: {connection}')
                return
        if self.selected_poi:
            poi = next((p for p in self.source.pois if p['id'] == self.selected_poi), None)
            if poi:
                status = 'Reported to GCS' if poi['reported_s'] is not None else 'Awaiting report'
                self.detail.config(text=f"{poi['id']}  •  priority {poi['priority']}\n"
                                        f"{status}\nPosition: {poi['x']:.0f}, {poi['y']:.0f} m\n"
                                        f"Spawned: {int(poi['spawn_s'])//60:02}:{int(poi['spawn_s'])%60:02}")
                return
        self.detail.config(text='Click an aircraft or PoI on the map or in the list.\nUse Follow to track a moving aircraft.')

    def draw(self):
        drones = self.source.drones
        hops, parents, _ = self.source.graph()
        if self.follow and self.selection_position() is not None:
            point = self.selection_position()
            x, y = self.xy(point)
            self.pan[0] += self.canvas.winfo_width()/2 - x
            self.pan[1] += self.canvas.winfo_height()/2 - y
        self._draw_map(drones, parents)
        data = self.source.summary()
        duration = self.source.scenario.mission_duration_s
        elapsed = int(self.source.time)
        if self.live:
            wall = int(self.source.wall_elapsed)
            if self.source.clock_at is None:
                clock_text = f'Gazebo waiting  |  Wall {wall//60:02}:{wall%60:02}'
            else:
                stalled = '  •  STALLED' if time.monotonic() - self.source.clock_at > 3 else ''
                clock_text = (f'Gazebo {elapsed//60:02}:{elapsed%60:02} / 45:00  |  '
                              f'Wall {wall//60:02}:{wall%60:02}  |  '
                              f'RTF {self.source.realtime_factor:.2f}×{stalled}')
        else:
            clock_text = f'{elapsed//60:02}:{elapsed%60:02} / 45:00  |  {self.speed:g}×'
        self.clock_label.config(text=clock_text)
        connected = sum(index in hops for index in range(1, len(drones)+1))
        separation = data['min_separation_m']
        sep_text = f'{separation:.1f} m' if math.isfinite(separation) else 'waiting'
        pending = sum(p['spawn_s'] <= self.source.time and p['reported_s'] is None for p in self.source.pois)
        self.stats.config(text=(f"PoIs reported       {data['reported']} / {data['poi_count']}    •    pending {pending}\n"
                                f'Connected aircraft  {connected} / {len(drones)}\n'
                                f'Minimum separation  {sep_text}\n'
                                f"Packet delivery     {data['pdr_pct']:.1f}%"))
        width = max(1, self.progress.winfo_width())
        self.progress.delete('all')
        self.progress.create_rectangle(0, 0, width*min(1, self.source.time/duration), 13,
                                       fill='#58d5eb', outline='')
        warnings = []
        if len(drones) < self.source.scenario.worst_case_aircraft():
            warnings.append(f'Fleet cannot span the full arena; need {self.source.scenario.worst_case_aircraft()} aircraft.')
        if data.get('violations') and any(data['violations'].values()):
            warnings.append('Constraint violation recorded; inspect the run export.')
        if self.live and (self.source.clock_at is None or time.monotonic()-self.source.clock_at > 3):
            warnings.append('Waiting for Gazebo clock updates.')
        if not warnings:
            warnings.append('Live telemetry • mission time follows Gazebo.' if self.live else
                            'Reference model • synthetic reports and ideal motion.')
        self.warning.config(text='\n'.join(warnings))
        self._refresh_lists(drones)
        self._refresh_detail(drones, hops)
        self.event_label.config(text='\n'.join(f'{int(t)//60:02}:{int(t)%60:02}  {message}'
                                               for t, message in self.source.events[-3:]))

    def tick(self):
        if self.closed:
            return
        now = time.monotonic()
        elapsed = min(now - self.last, 0.2)
        self.last = now
        if self.live:
            self.source.poll()
        elif self.running:
            self.accumulator += elapsed * self.speed
            deadline = time.monotonic() + 0.025
            while self.accumulator >= 0.5 and time.monotonic() < deadline:
                self.source.step(0.5)
                self.accumulator -= 0.5
        self.draw()
        self.after_id = self.root.after(100 if self.live else 33, self.tick)

    def close(self):
        if self.closed:
            return
        self.closed = True
        if self.after_id is not None:
            self.root.after_cancel(self.after_id)
        if self.live:
            self.source.close()
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--live',action='store_true');parser.add_argument('--fleet',type=int);parser.add_argument('--scenario')
    args,_=parser.parse_known_args();s=Scenario.load(args.scenario)
    Dashboard(LiveSwarm(s) if args.live else ReferenceSwarm(s,args.fleet),args.live).run()


if __name__=='__main__':main()
