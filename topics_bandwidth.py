#!/usr/bin/env python3
import argparse
import time
import threading
from collections import defaultdict, namedtuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.serialization import serialize_message
from rosidl_runtime_py.utilities import get_message

Stat = namedtuple('Stat', 'bytes msgs')

class TopicBandwidthMonitor(Node):
    def __init__(self, period, refresh_sec, include_hidden, exclude, qos_profile):
        super().__init__('topic_bandwidth_monitor_all')
        self.period = period
        self.refresh_sec = refresh_sec
        self.include_hidden = include_hidden
        self.exclude = exclude or []
        self.qos_profile = qos_profile

        self._lock = threading.Lock()
        # per-period counters
        self._period_stats = defaultdict(lambda: Stat(0, 0))
        # totals (since start)
        self._total_stats = defaultdict(lambda: Stat(0, 0))

        self._subs = {}   # topic -> subscription
        self._types = {}  # topic -> type string

        # timers
        self._print_timer = self.create_timer(self.period, self._print_table)
        self._refresh_timer = self.create_timer(self.refresh_sec, self._refresh_topics)

        # initial discovery
        self._refresh_topics(initial=True)

    # ---------- Discovery & subscription ----------
    def _refresh_topics(self, initial=False):
        try:
            topics = self.get_topic_names_and_types(no_demangle=False)
        except Exception as e:
            self.get_logger().warn(f'Failed to get topics: {e}')
            return

        for name, types in topics:
            if not self.include_hidden and name.startswith('_'):
                continue
            if any(pat in name for pat in self.exclude):
                continue
            if name in self._subs:
                continue
            # pick first type if multiple
            if not types:
                continue
            type_str = types[0]
            try:
                msg_cls = get_message(type_str)
            except Exception as e:
                self.get_logger().warn(f'Cannot import type {type_str} for {name}: {e}')
                continue

            try:
                sub = self.create_subscription(
                    msg_cls,
                    name,
                    self._make_cb(name),
                    self.qos_profile
                )
                self._subs[name] = sub
                self._types[name] = type_str
                self.get_logger().info(f'Subscribed: {name} [{type_str}]')
            except Exception as e:
                self.get_logger().warn(f'Failed to subscribe {name}: {e}')

        if initial and not self._subs:
            self.get_logger().warn('No topics found to subscribe to. Are publishers active?')

    def _make_cb(self, topic):
        def _cb(msg):
            try:
                size = len(serialize_message(msg))
            except Exception:
                # Fallback if serialization fails (shouldn’t in normal cases)
                size = 0
            with self._lock:
                ps = self._period_stats[topic]
                ts = self._total_stats[topic]
                self._period_stats[topic] = Stat(ps.bytes + size, ps.msgs + 1)
                self._total_stats[topic] = Stat(ts.bytes + size, ts.msgs + 1)
        return _cb

    # ---------- Reporting ----------
    def _print_table(self):
        now = time.time()
        with self._lock:
            # snapshot & reset period counters
            snap = dict(self._period_stats)
            self._period_stats = defaultdict(lambda: Stat(0, 0))
            totals = dict(self._total_stats)

        if not snap:
            self.get_logger().info('No messages received in this interval.')
            return

        # compute and sort by bandwidth (descending)
        rows = []
        interval = self.period
        total_bw_bits = 0.0
        for topic, stat in snap.items():
            if stat.msgs == 0:
                continue
            bw_mbps = (stat.bytes * 8.0) / interval / 1e6
            rate_hz = stat.msgs / interval
            avg_size = stat.bytes / stat.msgs if stat.msgs else 0
            total_bw_bits += stat.bytes * 8.0
            rows.append((
                bw_mbps,     # sort key
                topic,
                self._types.get(topic, '?'),
                f'{bw_mbps:8.3f}',
                f'{rate_hz:8.2f}',
                f'{int(avg_size):8d}',
                f'{totals.get(topic, Stat(0,0)).msgs:8d}'
            ))

        rows.sort(key=lambda r: r[0], reverse=True)

        # pretty print
        header = (
            '\n' +
            '='*94 + '\n' +
            f'ROS 2 Topic Bandwidth (interval {interval:.1f}s) @ {time.strftime("%Y-%m-%d %H:%M:%S")}\n' +
            '-'*94 + '\n' +
            f'{"BW [Mb/s]":>10}  {"Rate [Hz]":>10}  {"AvgSize [B]":>12}  {"Total Msgs":>10}  Topic (Type)\n' +
            '-'*94
        )
        lines = [header]
        for _, topic, type_str, bw, rate, avg, total in rows:
            lines.append(f'{bw:>10}  {rate:>10}  {avg:>12}  {total:>10}  {topic} ({type_str})')
        total_mbps = total_bw_bits / interval / 1e6
        lines.append('-'*94)
        lines.append(f'TOTAL: {total_mbps:.3f} Mb/s across {len(rows)} active topics')
        lines.append('='*94)
        print('\n'.join(lines))

def make_qos(depth, reliability, durability, history):
    qos = QoSProfile(depth=depth)
    qos.reliability = ReliabilityPolicy.BEST_EFFORT if reliability == 'best_effort' else ReliabilityPolicy.RELIABLE
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL if durability == 'transient_local' else DurabilityPolicy.VOLATILE
    qos.history = HistoryPolicy.KEEP_ALL if history == 'keep_all' else HistoryPolicy.KEEP_LAST
    return qos

def main():
    parser = argparse.ArgumentParser(description='Monitor bandwidth of ALL ROS 2 topics.')
    parser.add_argument('--period', type=float, default=2.0, help='Print interval seconds (default: 2.0)')
    parser.add_argument('--refresh', type=float, default=5.0, help='Topic discovery refresh seconds (default: 5.0)')
    parser.add_argument('--include-hidden', action='store_true', help='Include hidden topics (names starting with _)')
    parser.add_argument('--exclude', nargs='*', default=['/parameter_events', '/rosout'], help='Substring filters to exclude')
    parser.add_argument('--qos-depth', type=int, default=10, help='QoS queue depth (default: 10)')
    parser.add_argument('--qos-reliability', choices=['reliable', 'best_effort'], default='reliable', help='QoS reliability')
    parser.add_argument('--qos-durability', choices=['volatile', 'transient_local'], default='volatile', help='QoS durability')
    parser.add_argument('--qos-history', choices=['keep_last', 'keep_all'], default='keep_last', help='QoS history')
    args = parser.parse_args()

    rclpy.init()
    qos = make_qos(args.qos_depth, args.qos_reliability, args.qos_durability, args.qos_history)
    node = TopicBandwidthMonitor(
        period=args.period,
        refresh_sec=args.refresh,
        include_hidden=args.include_hidden,
        exclude=args.exclude,
        qos_profile=qos
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
