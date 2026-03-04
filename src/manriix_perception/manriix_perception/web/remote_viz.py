#!/usr/bin/env python3

from flask import Flask, render_template_string, jsonify, Response
import threading
import json

app = Flask(__name__)

# Configuration
CONFIG = {
    'rosbridge_port': 9090,
    'web_port': 5002,
    'safe_places': [
        {'name': 'Home Base', 'x': 0.0, 'y': 0.0},
        {'name': 'Charging Station', 'x': 2.0, 'y': -1.0},
        {'name': 'Exit Point', 'x': -3.0, 'y': 5.0}
    ]
}

# HTML Template - Complete 3-page visualizer
HTML_TEMPLATE = '''<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>MANRIIX NAVIGATION GUIDANCE SYSTEM</title>
    <script src="https://cdn.jsdelivr.net/npm/roslib@1/build/roslib.min.js"></script>
    <style>
        * { box-sizing: border-box; margin: 0; padding: 0; }

        :root {
            --bg-primary: #0d1117;
            --bg-secondary: #161b22;
            --bg-tertiary: #21262d;
            --border-color: #30363d;
            --text-primary: #f0f6fc;
            --text-secondary: #8b949e;
            --accent-yellow: #f0c000;
            --accent-green: #3fb950;
            --accent-red: #f85149;
            --accent-blue: #58a6ff;
            --accent-purple: #a371f7;
            --accent-cyan: #39c5cf;
            --accent-orange: #d29922;
        }

        body {
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
            background: var(--bg-primary);
            color: var(--text-primary);
            min-height: 100vh;
            overflow-x: hidden;
        }

        /* Header */
        .header {
            background: var(--bg-secondary);
            border-bottom: 1px solid var(--border-color);
            padding: 8px 16px;
            display: flex;
            justify-content: space-between;
            align-items: center;
            position: sticky;
            top: 0;
            z-index: 1000;
        }

        .header-left {
            display: flex;
            align-items: center;
        }

        .logo {
            font-size: 1.1em;
            font-weight: 700;
            color: var(--accent-yellow);
            letter-spacing: 0.5px;
        }

        .header-center {
            position: absolute;
            left: 50%;
            transform: translateX(-50%);
        }

        .nav-tabs {
            display: flex;
            gap: 4px;
        }

        .nav-tab {
            padding: 8px 16px;
            border: none;
            background: transparent;
            color: var(--text-secondary);
            font-size: 0.9em;
            font-weight: 500;
            cursor: pointer;
            border-radius: 6px;
            transition: all 0.2s;
        }

        .nav-tab:hover {
            background: var(--bg-tertiary);
            color: var(--text-primary);
        }

        .nav-tab.active {
            background: var(--accent-yellow);
            color: var(--bg-primary);
        }

        .header-right {
            display: flex;
            align-items: center;
            gap: 12px;
        }

        .stop-btn {
            padding: 8px 20px;
            background: var(--accent-red);
            color: white;
            border: none;
            border-radius: 6px;
            font-weight: 700;
            font-size: 0.9em;
            cursor: pointer;
            transition: all 0.2s;
        }

        .stop-btn:hover {
            background: #da3633;
            transform: scale(1.02);
        }

        .stop-btn:active {
            transform: scale(0.98);
        }

        .connection-status {
            display: flex;
            align-items: center;
            gap: 6px;
            font-size: 0.8em;
            color: var(--text-secondary);
        }

        .status-dot {
            width: 8px;
            height: 8px;
            border-radius: 50%;
            background: var(--accent-red);
        }

        .status-dot.connected {
            background: var(--accent-green);
        }

        /* Status Bar */
        .status-bar {
            background: var(--bg-secondary);
            border-bottom: 1px solid var(--border-color);
            padding: 10px 16px;
            display: flex;
            gap: 24px;
            align-items: center;
            flex-wrap: wrap;
        }

        .status-item {
            display: flex;
            align-items: center;
            gap: 8px;
            font-size: 0.85em;
        }

        .status-icon {
            font-size: 1.1em;
        }

        .status-value {
            font-weight: 600;
            color: var(--text-primary);
        }

        .status-label {
            color: var(--text-secondary);
        }

        .state-badge {
            padding: 4px 10px;
            border-radius: 12px;
            font-size: 0.75em;
            font-weight: 600;
            text-transform: uppercase;
        }

        .state-idle { background: var(--bg-tertiary); color: var(--text-secondary); }
        .state-evaluating { background: #3d2e00; color: var(--accent-yellow); }
        .state-positioning { background: #0c2d6b; color: var(--accent-blue); }
        .state-stabilizing { background: #3d2600; color: var(--accent-orange); }
        .state-capturing { background: #1a4d1a; color: var(--accent-green); }
        .state-moving { background: #0d3d3d; color: var(--accent-cyan); }
        .state-recovery { background: #4d1a1a; color: var(--accent-red); }

        /* Page Container */
        .page {
            display: none;
            padding: 12px;
            min-height: calc(100vh - 100px);
        }

        .page.active {
            display: block;
        }

        /* Page 1: Live View Layout - Cameras Left, Map Right */
        .live-layout {
            display: grid;
            grid-template-columns: 280px 1fr;
            gap: 12px;
            height: calc(100vh - 120px);
        }

        /* Left Column - Cameras stacked vertically */
        .cameras-column {
            display: flex;
            flex-direction: column;
            gap: 8px;
            overflow-y: auto;
        }

        .camera-card {
            background: var(--bg-secondary);
            border: 1px solid var(--border-color);
            border-radius: 8px;
            overflow: hidden;
            flex-shrink: 0;
        }

        .camera-card.maximized {
            position: fixed;
            top: 0;
            left: 0;
            right: 0;
            bottom: 0;
            z-index: 2000;
            border-radius: 0;
            display: flex;
            flex-direction: column;
        }

        .camera-card.maximized .camera-container {
            flex: 1;
            aspect-ratio: unset;
        }

        .camera-header {
            padding: 8px 12px;
            background: var(--bg-tertiary);
            display: flex;
            justify-content: space-between;
            align-items: center;
            font-size: 0.85em;
        }

        .camera-header-left {
            display: flex;
            align-items: center;
            gap: 8px;
        }

        .camera-label {
            font-weight: 600;
        }

        .camera-label.left { color: var(--accent-blue); }
        .camera-label.front { color: var(--accent-green); }
        .camera-label.right { color: var(--accent-purple); }

        .detection-count {
            color: var(--text-secondary);
            font-size: 0.85em;
        }

        .maximize-btn {
            background: transparent;
            border: 1px solid var(--border-color);
            color: var(--text-secondary);
            padding: 4px 8px;
            border-radius: 4px;
            cursor: pointer;
            font-size: 0.8em;
            transition: all 0.2s;
        }

        .maximize-btn:hover {
            background: var(--bg-primary);
            color: var(--text-primary);
        }

        .camera-container {
            position: relative;
            aspect-ratio: 16/9;
            background: var(--bg-primary);
            overflow: hidden;
        }

        .camera-container img {
            position: absolute;
            top: 0;
            left: 0;
            width: 100%;
            height: 100%;
            object-fit: contain;
            z-index: 1;
        }

        .camera-container canvas {
            position: absolute;
            top: 0;
            left: 0;
            width: 100%;
            height: 100%;
            pointer-events: none;
            z-index: 2;
        }

        .camera-placeholder {
            position: absolute;
            top: 0;
            left: 0;
            width: 100%;
            height: 100%;
            display: flex;
            align-items: center;
            justify-content: center;
            color: var(--text-secondary);
            font-size: 0.9em;
            z-index: 0;
        }

        /* Right Column - Map and Info Panels */
        .right-content {
            display: flex;
            flex-direction: column;
            gap: 12px;
            min-height: 0;
        }

        /* Map Panel */
        .map-panel {
            background: var(--bg-secondary);
            border: 1px solid var(--border-color);
            border-radius: 8px;
            display: flex;
            flex-direction: column;
            overflow: hidden;
            flex: 1;
            min-height: 0;
        }

        .map-panel.maximized {
            position: fixed;
            top: 0;
            left: 0;
            right: 0;
            bottom: 0;
            z-index: 2000;
            border-radius: 0;
        }

        .map-header {
            padding: 10px 12px;
            background: var(--bg-tertiary);
            display: flex;
            justify-content: space-between;
            align-items: center;
            border-bottom: 1px solid var(--border-color);
        }

        .map-header-left {
            display: flex;
            align-items: center;
            gap: 12px;
        }

        .map-title {
            font-weight: 600;
            font-size: 0.9em;
        }

        .map-layers {
            display: flex;
            gap: 6px;
        }

        .layer-toggle {
            padding: 4px 8px;
            border: 1px solid var(--border-color);
            background: transparent;
            color: var(--text-secondary);
            font-size: 0.75em;
            border-radius: 4px;
            cursor: pointer;
            transition: all 0.2s;
        }

        .layer-toggle.active {
            background: var(--accent-yellow);
            color: var(--bg-primary);
            border-color: var(--accent-yellow);
        }

        .map-canvas-container {
            flex: 1;
            position: relative;
            min-height: 200px;
        }

        #map-canvas {
            width: 100%;
            height: 100%;
        }

        .map-legend {
            padding: 8px 12px;
            background: var(--bg-tertiary);
            display: flex;
            gap: 16px;
            flex-wrap: wrap;
            font-size: 0.75em;
            color: var(--text-secondary);
            border-top: 1px solid var(--border-color);
        }

        .legend-item {
            display: flex;
            align-items: center;
            gap: 4px;
        }

        .legend-symbol {
            width: 12px;
            height: 12px;
            display: flex;
            align-items: center;
            justify-content: center;
            font-size: 10px;
        }

        /* Info Panels Row - Below Map */
        .info-panels-row {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 12px;
            flex-shrink: 0;
        }

        .panel-card {
            background: var(--bg-secondary);
            border: 1px solid var(--border-color);
            border-radius: 8px;
            overflow: hidden;
        }

        .panel-header {
            padding: 10px 12px;
            background: var(--bg-tertiary);
            font-weight: 600;
            font-size: 0.85em;
            border-bottom: 1px solid var(--border-color);
        }

        .panel-content {
            padding: 12px;
        }

        .info-grid {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 8px;
        }

        .info-item {
            display: flex;
            flex-direction: column;
            gap: 2px;
        }

        .info-label {
            font-size: 0.7em;
            color: var(--text-secondary);
            text-transform: uppercase;
        }

        .info-value {
            font-size: 0.95em;
            font-weight: 600;
            font-family: 'SF Mono', Monaco, monospace;
        }

        /* Photo Session Panel */
        .photo-status {
            display: flex;
            align-items: center;
            gap: 8px;
            margin-bottom: 12px;
        }

        .photo-indicator {
            width: 10px;
            height: 10px;
            border-radius: 50%;
            background: var(--text-secondary);
        }

        .photo-indicator.active {
            background: var(--accent-green);
            animation: pulse 1.5s infinite;
        }

        @keyframes pulse {
            0%, 100% { opacity: 1; }
            50% { opacity: 0.5; }
        }

        .photo-timing {
            display: flex;
            align-items: baseline;
            gap: 8px;
            margin-bottom: 8px;
        }

        .timing-assigned {
            font-size: 1.2em;
            font-weight: 600;
            color: var(--text-secondary);
        }

        .timing-divider {
            color: var(--text-secondary);
        }

        .timing-remaining {
            font-size: 1.5em;
            font-weight: 700;
            color: var(--accent-green);
        }

        .progress-bar {
            height: 6px;
            background: var(--bg-tertiary);
            border-radius: 3px;
            overflow: hidden;
        }

        .progress-fill {
            height: 100%;
            background: var(--accent-green);
            transition: width 0.3s;
        }

        /* Recovery Alert */
        .recovery-alert {
            display: none;
            background: #4d1a1a;
            border: 1px solid var(--accent-red);
            border-radius: 8px;
            padding: 12px;
        }

        .recovery-alert.active {
            display: block;
        }

        .recovery-header {
            display: flex;
            align-items: center;
            gap: 8px;
            margin-bottom: 8px;
            color: var(--accent-red);
            font-weight: 600;
        }

        .recovery-level {
            font-size: 0.9em;
            color: var(--text-primary);
        }

        .recovery-attempts {
            font-size: 0.8em;
            color: var(--text-secondary);
        }

        /* Emergency Stop Overlay */
        .stop-overlay {
            display: none;
            position: fixed;
            top: 0;
            left: 0;
            width: 100%;
            height: 100%;
            background: rgba(0, 0, 0, 0.9);
            z-index: 2000;
            justify-content: center;
            align-items: center;
        }

        .stop-overlay.active {
            display: flex;
        }

        .stop-panel {
            background: var(--bg-secondary);
            border: 2px solid var(--accent-red);
            border-radius: 12px;
            padding: 32px;
            max-width: 500px;
            width: 90%;
            text-align: center;
        }

        .stop-icon {
            font-size: 4em;
            margin-bottom: 16px;
        }

        .stop-title {
            font-size: 1.5em;
            font-weight: 700;
            color: var(--accent-red);
            margin-bottom: 16px;
        }

        .stop-info {
            color: var(--text-secondary);
            margin-bottom: 24px;
            line-height: 1.5;
        }

        .stop-position {
            font-family: 'SF Mono', Monaco, monospace;
            color: var(--text-primary);
        }

        .stop-actions {
            display: flex;
            flex-direction: column;
            gap: 12px;
        }

        .action-btn {
            padding: 16px 24px;
            border: none;
            border-radius: 8px;
            font-size: 1em;
            font-weight: 600;
            cursor: pointer;
            transition: all 0.2s;
            display: flex;
            flex-direction: column;
            align-items: center;
            gap: 4px;
        }

        .action-btn .btn-subtitle {
            font-size: 0.75em;
            font-weight: 400;
            opacity: 0.8;
        }

        .btn-resume {
            background: var(--accent-green);
            color: white;
        }

        .btn-resume:hover {
            background: #2ea043;
        }

        .btn-safe {
            background: var(--accent-blue);
            color: white;
        }

        .btn-safe:hover {
            background: #4393e6;
        }

        .btn-stay {
            background: var(--bg-tertiary);
            color: var(--text-primary);
            border: 1px solid var(--border-color);
        }

        .btn-stay:hover {
            background: var(--border-color);
        }

        .safe-place-select {
            width: 100%;
            padding: 8px;
            margin-top: 8px;
            background: var(--bg-tertiary);
            border: 1px solid var(--border-color);
            border-radius: 4px;
            color: var(--text-primary);
            font-size: 0.9em;
        }

        .hold-progress {
            height: 4px;
            background: var(--bg-tertiary);
            border-radius: 2px;
            margin-top: 8px;
            overflow: hidden;
        }

        .hold-progress-fill {
            height: 100%;
            width: 0%;
            background: var(--accent-green);
            transition: width 0.1s linear;
        }

        /* Page 2: Analytics */
        .analytics-layout {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 12px;
        }

        .analytics-card {
            background: var(--bg-secondary);
            border: 1px solid var(--border-color);
            border-radius: 8px;
        }

        .analytics-header {
            padding: 12px 16px;
            background: var(--bg-tertiary);
            font-weight: 600;
            font-size: 0.9em;
            border-bottom: 1px solid var(--border-color);
            display: flex;
            align-items: center;
            gap: 8px;
        }

        .analytics-content {
            padding: 16px;
        }

        .metric-grid {
            display: grid;
            grid-template-columns: repeat(2, 1fr);
            gap: 16px;
        }

        .metric-item {
            text-align: center;
        }

        .metric-value {
            font-size: 1.8em;
            font-weight: 700;
            color: var(--accent-yellow);
        }

        .metric-label {
            font-size: 0.75em;
            color: var(--text-secondary);
            margin-top: 4px;
        }

        .score-bar {
            display: flex;
            align-items: center;
            gap: 12px;
            margin-bottom: 12px;
        }

        .score-label {
            width: 100px;
            font-size: 0.8em;
            color: var(--text-secondary);
        }

        .score-track {
            flex: 1;
            height: 8px;
            background: var(--bg-tertiary);
            border-radius: 4px;
            overflow: hidden;
        }

        .score-fill {
            height: 100%;
            border-radius: 4px;
            transition: width 0.3s;
        }

        .score-fill.aesthetic { background: var(--accent-purple); }
        .score-fill.accessibility { background: var(--accent-blue); }
        .score-fill.stability { background: var(--accent-green); }
        .score-fill.social { background: var(--accent-orange); }

        .score-value {
            width: 40px;
            text-align: right;
            font-size: 0.85em;
            font-weight: 600;
        }

        .poi-list {
            font-size: 0.85em;
        }

        .poi-item {
            display: grid;
            grid-template-columns: 30px 1fr 60px 80px;
            gap: 8px;
            padding: 8px 0;
            border-bottom: 1px solid var(--border-color);
            align-items: center;
        }

        .poi-item:last-child {
            border-bottom: none;
        }

        .poi-rank {
            font-weight: 600;
            color: var(--accent-yellow);
        }

        .poi-position {
            font-family: 'SF Mono', Monaco, monospace;
            font-size: 0.85em;
        }

        .poi-score {
            font-weight: 600;
        }

        .poi-formation {
            font-size: 0.75em;
            color: var(--text-secondary);
        }

        .formation-chart {
            display: flex;
            flex-direction: column;
            gap: 8px;
        }

        .formation-bar {
            display: flex;
            align-items: center;
            gap: 12px;
        }

        .formation-label {
            width: 100px;
            font-size: 0.8em;
        }

        .formation-track {
            flex: 1;
            height: 16px;
            background: var(--bg-tertiary);
            border-radius: 4px;
            overflow: hidden;
        }

        .formation-fill {
            height: 100%;
            background: var(--accent-cyan);
            display: flex;
            align-items: center;
            justify-content: flex-end;
            padding-right: 8px;
            font-size: 0.7em;
            font-weight: 600;
            color: var(--bg-primary);
            min-width: fit-content;
        }

        /* Page 3: System */
        .system-layout {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 12px;
        }

        .resource-meter {
            margin-bottom: 16px;
        }

        .resource-header {
            display: flex;
            justify-content: space-between;
            margin-bottom: 6px;
            font-size: 0.85em;
        }

        .resource-label {
            color: var(--text-secondary);
        }

        .resource-value {
            font-weight: 600;
        }

        .resource-bar {
            height: 8px;
            background: var(--bg-tertiary);
            border-radius: 4px;
            overflow: hidden;
        }

        .resource-fill {
            height: 100%;
            background: var(--accent-green);
            transition: width 0.3s;
        }

        .resource-fill.warning { background: var(--accent-orange); }
        .resource-fill.danger { background: var(--accent-red); }

        .component-list {
            font-size: 0.85em;
        }

        .component-item {
            display: flex;
            justify-content: space-between;
            align-items: center;
            padding: 8px 0;
            border-bottom: 1px solid var(--border-color);
        }

        .component-item:last-child {
            border-bottom: none;
        }

        .component-name {
            display: flex;
            align-items: center;
            gap: 8px;
        }

        .component-status {
            padding: 2px 8px;
            border-radius: 10px;
            font-size: 0.75em;
            font-weight: 600;
        }

        .component-status.active {
            background: #1a4d1a;
            color: var(--accent-green);
        }

        .component-status.standby {
            background: #3d2e00;
            color: var(--accent-yellow);
        }

        .component-status.idle {
            background: var(--bg-tertiary);
            color: var(--text-secondary);
        }

        .component-status.error {
            background: #4d1a1a;
            color: var(--accent-red);
        }

        .recovery-levels {
            display: flex;
            gap: 4px;
            margin: 16px 0;
        }

        .recovery-level-box {
            flex: 1;
            height: 24px;
            background: var(--bg-tertiary);
            border-radius: 4px;
            display: flex;
            align-items: center;
            justify-content: center;
            font-size: 0.7em;
            font-weight: 600;
            color: var(--text-secondary);
        }

        .recovery-level-box.current {
            background: var(--accent-yellow);
            color: var(--bg-primary);
        }

        .recovery-level-box.passed {
            background: var(--accent-green);
            color: white;
        }

        .recovery-stats {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            gap: 16px;
            text-align: center;
            margin-top: 16px;
        }

        .recovery-stat-value {
            font-size: 1.5em;
            font-weight: 700;
        }

        .recovery-stat-label {
            font-size: 0.7em;
            color: var(--text-secondary);
        }

        .event-log {
            max-height: 250px;
            overflow-y: auto;
            font-family: 'SF Mono', Monaco, monospace;
            font-size: 0.8em;
        }

        .event-item {
            padding: 6px 0;
            border-bottom: 1px solid var(--border-color);
            display: flex;
            gap: 12px;
        }

        .event-time {
            color: var(--text-secondary);
            white-space: nowrap;
        }

        .event-msg {
            color: var(--text-primary);
        }

        .event-msg.success { color: var(--accent-green); }
        .event-msg.warning { color: var(--accent-orange); }
        .event-msg.error { color: var(--accent-red); }

        .performance-grid {
            display: grid;
            grid-template-columns: repeat(2, 1fr);
            gap: 12px;
        }

        .perf-item {
            background: var(--bg-tertiary);
            padding: 12px;
            border-radius: 6px;
            text-align: center;
        }

        .perf-value {
            font-size: 1.3em;
            font-weight: 700;
            color: var(--accent-cyan);
        }

        .perf-label {
            font-size: 0.7em;
            color: var(--text-secondary);
            margin-top: 4px;
        }

        /* Responsive */
        @media (max-width: 1200px) {
            .live-layout {
                grid-template-columns: 240px 1fr;
            }
            .analytics-layout,
            .system-layout {
                grid-template-columns: 1fr;
            }
            .header-center {
                position: static;
                transform: none;
            }
            .header {
                flex-wrap: wrap;
                gap: 8px;
            }
        }

        @media (max-width: 900px) {
            .live-layout {
                grid-template-columns: 1fr;
                grid-template-rows: auto 1fr;
            }
            .cameras-column {
                flex-direction: row;
                overflow-x: auto;
                gap: 8px;
            }
            .camera-card {
                min-width: 280px;
            }
            .info-panels-row {
                grid-template-columns: 1fr;
            }
            .header-center {
                order: 3;
                width: 100%;
                justify-content: center;
                display: flex;
            }
        }
    </style>
</head>
<body>
    <!-- Header -->
    <header class="header">
        <div class="header-left">
            <div class="logo">MANRIIX NAVIGATION GUIDANCE SYSTEM</div>
        </div>
        <div class="header-center">
            <nav class="nav-tabs">
                <button class="nav-tab active" onclick="showPage('live')">LIVE</button>
                <button class="nav-tab" onclick="showPage('analytics')">ANALYTICS</button>
                <button class="nav-tab" onclick="showPage('system')">SYSTEM</button>
            </nav>
        </div>
        <div class="header-right">
            <div class="connection-status">
                <span class="status-dot" id="ros-status-dot"></span>
                <span id="ros-status-text">Disconnected</span>
            </div>
            <button class="stop-btn" onclick="emergencyStop()">STOP</button>
        </div>
    </header>

    <!-- Status Bar -->
    <div class="status-bar">
        <div class="status-item">
            <span class="status-icon">👥</span>
            <span class="status-value" id="humans-count">0</span>
            <span class="status-label">Humans</span>
        </div>
        <div class="status-item">
            <span class="status-icon">🔵</span>
            <span class="status-value" id="clusters-count">0</span>
            <span class="status-label">Clusters</span>
        </div>
        <div class="status-item">
            <span class="status-icon">🤖</span>
            <span class="status-label">State:</span>
            <span class="state-badge state-idle" id="mission-state">IDLE</span>
        </div>
    </div>

    <!-- Page 1: Live View -->
    <div class="page active" id="page-live">
        <div class="live-layout">
            <!-- Left Column: Cameras -->
            <div class="cameras-column">
                <div class="camera-card" id="camera-card-left">
                    <div class="camera-header">
                        <div class="camera-header-left">
                            <span class="camera-label left">LEFT</span>
                            <span class="detection-count">👤 <span id="det-left">0</span></span>
                        </div>
                        <button class="maximize-btn" onclick="toggleMaximize('camera-card-left')">⛶</button>
                    </div>
                    <div class="camera-container">
                        <div class="camera-placeholder" id="placeholder-left">Waiting for feed...</div>
                        <img id="img-left" alt="Left Camera">
                        <canvas id="bbox-left"></canvas>
                    </div>
                </div>
                <div class="camera-card" id="camera-card-front">
                    <div class="camera-header">
                        <div class="camera-header-left">
                            <span class="camera-label front">FRONT</span>
                            <span class="detection-count">👤 <span id="det-front">0</span></span>
                        </div>
                        <button class="maximize-btn" onclick="toggleMaximize('camera-card-front')">⛶</button>
                    </div>
                    <div class="camera-container">
                        <div class="camera-placeholder" id="placeholder-front">Waiting for feed...</div>
                        <img id="img-front" alt="Front Camera">
                        <canvas id="bbox-front"></canvas>
                    </div>
                </div>
                <div class="camera-card" id="camera-card-right">
                    <div class="camera-header">
                        <div class="camera-header-left">
                            <span class="camera-label right">RIGHT</span>
                            <span class="detection-count">👤 <span id="det-right">0</span></span>
                        </div>
                        <button class="maximize-btn" onclick="toggleMaximize('camera-card-right')">⛶</button>
                    </div>
                    <div class="camera-container">
                        <div class="camera-placeholder" id="placeholder-right">Waiting for feed...</div>
                        <img id="img-right" alt="Right Camera">
                        <canvas id="bbox-right"></canvas>
                    </div>
                </div>
            </div>

            <!-- Right Column: Map + Info Panels -->
            <div class="right-content">
                <!-- Map Panel -->
                <div class="map-panel" id="map-panel">
                    <div class="map-header">
                        <div class="map-header-left">
                            <span class="map-title">MAP VIEW</span>
                            <div class="map-layers">
                                <button class="layer-toggle active" onclick="toggleLayer('laser')" id="layer-laser">Laser</button>
                                <button class="layer-toggle active" onclick="toggleLayer('path')" id="layer-path">Path</button>
                                <button class="layer-toggle active" onclick="toggleLayer('costmap')" id="layer-costmap">Costmap</button>
                                <button class="layer-toggle active" onclick="toggleLayer('frontiers')" id="layer-frontiers">Frontiers</button>
                                <button class="layer-toggle active" onclick="toggleLayer('pois')" id="layer-pois">POIs</button>
                            </div>
                        </div>
                        <button class="maximize-btn" onclick="toggleMaximize('map-panel')">⛶</button>
                    </div>
                    <div class="map-canvas-container">
                        <canvas id="map-canvas"></canvas>
                    </div>
                    <div class="map-legend">
                        <div class="legend-item"><span class="legend-symbol" style="color: var(--accent-green);">▲</span> Robot</div>
                        <div class="legend-item"><span class="legend-symbol" style="color: var(--accent-red);">●</span> Human</div>
                        <div class="legend-item"><span class="legend-symbol" style="color: var(--accent-cyan);">○</span> Cluster</div>
                        <div class="legend-item"><span class="legend-symbol" style="color: var(--accent-yellow);">★</span> Optimal</div>
                        <div class="legend-item"><span class="legend-symbol" style="color: var(--accent-purple);">◊</span> Frontier</div>
                    </div>
                </div>

                <!-- Info Panels Row: Robot State + Photo Session -->
                <div class="info-panels-row">
                    <!-- Robot State -->
                    <div class="panel-card">
                        <div class="panel-header">ROBOT STATE</div>
                        <div class="panel-content">
                            <div class="info-grid">
                                <div class="info-item">
                                    <span class="info-label">X</span>
                                    <span class="info-value" id="robot-x">0.00 m</span>
                                </div>
                                <div class="info-item">
                                    <span class="info-label">Y</span>
                                    <span class="info-value" id="robot-y">0.00 m</span>
                                </div>
                                <div class="info-item">
                                    <span class="info-label">θ</span>
                                    <span class="info-value" id="robot-theta">0.0°</span>
                                </div>
                                <div class="info-item">
                                    <span class="info-label">Speed</span>
                                    <span class="info-value" id="robot-speed">0.00 m/s</span>
                                </div>
                            </div>
                        </div>
                    </div>

                    <!-- Photo Session -->
                    <div class="panel-card">
                        <div class="panel-header">PHOTO SESSION</div>
                        <div class="panel-content">
                            <div class="photo-status">
                                <span class="photo-indicator" id="photo-indicator"></span>
                                <span id="photo-status-text">Idle</span>
                            </div>
                            <div class="photo-timing">
                                <span class="timing-assigned" id="photo-assigned">--s</span>
                                <span class="timing-divider">│</span>
                                <span class="timing-remaining" id="photo-remaining">--</span>
                            </div>
                            <div class="progress-bar">
                                <div class="progress-fill" id="photo-progress" style="width: 0%"></div>
                            </div>
                        </div>
                    </div>
                </div>

                <!-- Recovery Alert (hidden by default) -->
                <div class="recovery-alert" id="recovery-alert">
                    <div class="recovery-header">
                        <span>⚠️</span>
                        <span>RECOVERY ACTIVE</span>
                    </div>
                    <div class="recovery-level">Level <span id="recovery-level">0</span>: <span id="recovery-name">Normal</span></div>
                    <div class="recovery-attempts">Attempt <span id="recovery-attempts">0</span>/5</div>
                </div>
            </div>
        </div>
    </div>

    <!-- Page 2: Analytics -->
    <div class="page" id="page-analytics">
        <div class="analytics-layout">
            <!-- Mission Metrics -->
            <div class="analytics-card">
                <div class="analytics-header">📊 MISSION METRICS</div>
                <div class="analytics-content">
                    <div class="metric-grid">
                        <div class="metric-item">
                            <div class="metric-value" id="metric-photos">0</div>
                            <div class="metric-label">Photos Completed</div>
                        </div>
                        <div class="metric-item">
                            <div class="metric-value" id="metric-targets">0</div>
                            <div class="metric-label">Targets Processed</div>
                        </div>
                        <div class="metric-item">
                            <div class="metric-value" id="metric-distance">0</div>
                            <div class="metric-label">Distance (m)</div>
                        </div>
                        <div class="metric-item">
                            <div class="metric-value" id="metric-efficiency">0%</div>
                            <div class="metric-label">Efficiency</div>
                        </div>
                    </div>
                </div>
            </div>

            <!-- Photo Intelligence -->
            <div class="analytics-card">
                <div class="analytics-header">📷 PHOTO INTELLIGENCE</div>
                <div class="analytics-content">
                    <div class="metric-grid">
                        <div class="metric-item">
                            <div class="metric-value" id="intel-group-size">0</div>
                            <div class="metric-label">Group Size</div>
                        </div>
                        <div class="metric-item">
                            <div class="metric-value" id="intel-density">0.0</div>
                            <div class="metric-label">Density (/m²)</div>
                        </div>
                        <div class="metric-item">
                            <div class="metric-value" id="intel-stability">0%</div>
                            <div class="metric-label">Stability</div>
                        </div>
                        <div class="metric-item">
                            <div class="metric-value" id="intel-readiness">0%</div>
                            <div class="metric-label">Readiness</div>
                        </div>
                    </div>
                </div>
            </div>

            <!-- POI Analysis - 10-Factor Scoring System -->
            <div class="analytics-card">
                <div class="analytics-header">🎯 POI SCORING FACTORS (10-Factor System)</div>
                <div class="analytics-content" id="poi-factors-content">
                    <div class="score-bar">
                        <span class="score-label" title="Proximity to robot (20%)">Distance</span>
                        <div class="score-track"><div class="score-fill" id="score-distance" style="width: 20%; background: var(--accent-blue)"></div></div>
                        <span class="score-value">20%</span>
                    </div>
                    <div class="score-bar">
                        <span class="score-label" title="Group consistency (15%)">Composition</span>
                        <div class="score-track"><div class="score-fill" id="score-composition" style="width: 15%; background: var(--accent-purple)"></div></div>
                        <span class="score-value">15%</span>
                    </div>
                    <div class="score-bar">
                        <span class="score-label" title="Clear of obstacles (15%)">Obstacles</span>
                        <div class="score-track"><div class="score-fill" id="score-obstacles" style="width: 15%; background: var(--accent-orange)"></div></div>
                        <span class="score-value">15%</span>
                    </div>
                    <div class="score-bar">
                        <span class="score-label" title="Cluster size (10%)">Size</span>
                        <div class="score-track"><div class="score-fill" id="score-size" style="width: 10%; background: var(--accent-green)"></div></div>
                        <span class="score-value">10%</span>
                    </div>
                    <div class="score-bar">
                        <span class="score-label" title="Crowd density (10%)">Density</span>
                        <div class="score-track"><div class="score-fill" id="score-density" style="width: 10%; background: var(--accent-cyan)"></div></div>
                        <span class="score-value">10%</span>
                    </div>
                    <div class="score-bar">
                        <span class="score-label" title="Group arrangement (10%)">Formation</span>
                        <div class="score-track"><div class="score-fill" id="score-formation" style="width: 10%; background: var(--accent-yellow)"></div></div>
                        <span class="score-value">10%</span>
                    </div>
                    <div class="score-bar">
                        <span class="score-label" title="Temporal stability (10%)">Stability</span>
                        <div class="score-track"><div class="score-fill" id="score-stability" style="width: 10%; background: var(--accent-green)"></div></div>
                        <span class="score-value">10%</span>
                    </div>
                    <div class="score-bar">
                        <span class="score-label" title="Navigation ease (10%)">Env Complex</span>
                        <div class="score-track"><div class="score-fill" id="score-env" style="width: 10%; background: var(--accent-blue)"></div></div>
                        <span class="score-value">10%</span>
                    </div>
                    <div class="score-bar">
                        <span class="score-label" title="Path clearance (10%)">Path Clear</span>
                        <div class="score-track"><div class="score-fill" id="score-path" style="width: 10%; background: var(--accent-purple)"></div></div>
                        <span class="score-value">10%</span>
                    </div>
                    <div class="score-bar">
                        <span class="score-label" title="Position history (5%)">Temporal</span>
                        <div class="score-track"><div class="score-fill" id="score-temporal" style="width: 5%; background: var(--accent-orange)"></div></div>
                        <span class="score-value">5%</span>
                    </div>
                    <div style="margin-top: 12px; padding-top: 8px; border-top: 1px solid var(--border-color);">
                        <div style="display: flex; justify-content: space-between; font-size: 0.85em;">
                            <span style="color: var(--text-secondary);">Avg Priority Score:</span>
                            <span id="avg-priority-score" style="font-weight: 600; color: var(--accent-yellow);">--</span>
                        </div>
                        <div style="display: flex; justify-content: space-between; font-size: 0.85em; margin-top: 4px;">
                            <span style="color: var(--text-secondary);">Active POIs:</span>
                            <span id="num-pois" style="font-weight: 600;">--</span>
                        </div>
                    </div>
                </div>
            </div>

            <!-- Top POIs -->
            <div class="analytics-card">
                <div class="analytics-header">📍 TOP POIs</div>
                <div class="analytics-content">
                    <div class="poi-list" id="poi-list">
                        <div class="poi-item">
                            <span class="poi-rank">#1</span>
                            <span class="poi-position">--</span>
                            <span class="poi-score">--</span>
                            <span class="poi-formation">--</span>
                        </div>
                    </div>
                </div>
            </div>

            <!-- Cluster Analysis -->
            <div class="analytics-card" style="grid-column: span 2;">
                <div class="analytics-header">🔵 CLUSTER FORMATIONS</div>
                <div class="analytics-content">
                    <div class="formation-chart" id="formation-chart">
                        <div class="formation-bar">
                            <span class="formation-label">Pairs</span>
                            <div class="formation-track"><div class="formation-fill" style="width: 0%">0</div></div>
                        </div>
                    </div>
                </div>
            </div>
        </div>
    </div>

    <!-- Page 3: System -->
    <div class="page" id="page-system">
        <div class="system-layout">
            <!-- System Resources -->
            <div class="analytics-card">
                <div class="analytics-header">💻 SYSTEM RESOURCES</div>
                <div class="analytics-content">
                    <div class="resource-meter">
                        <div class="resource-header">
                            <span class="resource-label">CPU</span>
                            <span class="resource-value" id="cpu-value">0%</span>
                        </div>
                        <div class="resource-bar"><div class="resource-fill" id="cpu-bar" style="width: 0%"></div></div>
                    </div>
                    <div class="resource-meter">
                        <div class="resource-header">
                            <span class="resource-label">Memory</span>
                            <span class="resource-value" id="mem-value">0%</span>
                        </div>
                        <div class="resource-bar"><div class="resource-fill" id="mem-bar" style="width: 0%"></div></div>
                    </div>
                    <div class="resource-meter">
                        <div class="resource-header">
                            <span class="resource-label">Disk</span>
                            <span class="resource-value" id="disk-value">0%</span>
                        </div>
                        <div class="resource-bar"><div class="resource-fill" id="disk-bar" style="width: 0%"></div></div>
                    </div>
                </div>
            </div>

            <!-- Component Status -->
            <div class="analytics-card">
                <div class="analytics-header">🔧 COMPONENT STATUS</div>
                <div class="analytics-content">
                    <div class="component-list" id="component-list">
                        <div class="component-item">
                            <span class="component-name">● ZED Fusion</span>
                            <span class="component-status idle">Checking...</span>
                        </div>
                        <div class="component-item">
                            <span class="component-name">● Human Clustering</span>
                            <span class="component-status idle">Checking...</span>
                        </div>
                        <div class="component-item">
                            <span class="component-name">● POI Manager</span>
                            <span class="component-status idle">Checking...</span>
                        </div>
                        <div class="component-item">
                            <span class="component-name">● Mission Controller</span>
                            <span class="component-status idle">Checking...</span>
                        </div>
                        <div class="component-item">
                            <span class="component-name">● Photo Intelligence</span>
                            <span class="component-status idle">Checking...</span>
                        </div>
                        <div class="component-item">
                            <span class="component-name">● Recovery Manager</span>
                            <span class="component-status idle">Checking...</span>
                        </div>
                    </div>
                </div>
            </div>

            <!-- Recovery System -->
            <div class="analytics-card">
                <div class="analytics-header">🛡️ RECOVERY SYSTEM</div>
                <div class="analytics-content">
                    <div style="margin-bottom: 12px;">Current Level: <strong id="sys-recovery-level">0 (NORMAL)</strong></div>
                    <div class="recovery-levels" id="recovery-levels">
                        <div class="recovery-level-box current">0</div>
                        <div class="recovery-level-box">1</div>
                        <div class="recovery-level-box">2</div>
                        <div class="recovery-level-box">3</div>
                        <div class="recovery-level-box">4</div>
                        <div class="recovery-level-box">5</div>
                        <div class="recovery-level-box">6</div>
                        <div class="recovery-level-box">7</div>
                        <div class="recovery-level-box">8</div>
                    </div>
                    <div class="recovery-stats">
                        <div>
                            <div class="recovery-stat-value" id="recovery-sessions">0</div>
                            <div class="recovery-stat-label">Sessions</div>
                        </div>
                        <div>
                            <div class="recovery-stat-value" id="recovery-success">0</div>
                            <div class="recovery-stat-label">Successful</div>
                        </div>
                        <div>
                            <div class="recovery-stat-value" id="recovery-avg-time">0s</div>
                            <div class="recovery-stat-label">Avg Time</div>
                        </div>
                    </div>
                </div>
            </div>

            <!-- Performance -->
            <div class="analytics-card">
                <div class="analytics-header">⚡ PERFORMANCE</div>
                <div class="analytics-content">
                    <div class="performance-grid">
                        <div class="perf-item">
                            <div class="perf-value" id="perf-clustering">0 Hz</div>
                            <div class="perf-label">Clustering Rate</div>
                        </div>
                        <div class="perf-item">
                            <div class="perf-value" id="perf-detection">0 Hz</div>
                            <div class="perf-label">Detection Rate</div>
                        </div>
                        <div class="perf-item">
                            <div class="perf-value" id="perf-poi">0 Hz</div>
                            <div class="perf-label">POI Calc Rate</div>
                        </div>
                        <div class="perf-item">
                            <div class="perf-value" id="perf-errors">0</div>
                            <div class="perf-label">Errors</div>
                        </div>
                    </div>
                </div>
            </div>

            <!-- Event Log -->
            <div class="analytics-card" style="grid-column: span 2;">
                <div class="analytics-header">📜 RECENT EVENTS</div>
                <div class="analytics-content">
                    <div class="event-log" id="event-log">
                        <div class="event-item">
                            <span class="event-time">--:--:--</span>
                            <span class="event-msg">Waiting for events...</span>
                        </div>
                    </div>
                </div>
            </div>
        </div>
    </div>

    <!-- Emergency Stop Overlay -->
    <div class="stop-overlay" id="stop-overlay">
        <div class="stop-panel">
            <div class="stop-icon">🛑</div>
            <div class="stop-title">EMERGENCY STOP ACTIVE</div>
            <div class="stop-info">
                Robot stopped at: <span class="stop-position" id="stop-position">x: 0.00, y: 0.00, θ: 0°</span>
                <br><br>
                Previous goal: <span class="stop-position" id="stop-goal">x: 0.00, y: 0.00</span>
            </div>
            <div class="stop-actions">
                <button class="action-btn btn-resume" id="btn-resume"
                        onmousedown="startHold('resume')" onmouseup="cancelHold()" onmouseleave="cancelHold()"
                        ontouchstart="startHold('resume')" ontouchend="cancelHold()">
                    🔄 RESUME PREVIOUS GOAL
                    <span class="btn-subtitle">Hold 2 seconds to resume</span>
                    <div class="hold-progress"><div class="hold-progress-fill" id="resume-progress"></div></div>
                </button>
                <button class="action-btn btn-safe" onclick="goToSafePlace()">
                    🏠 GO TO SAFE PLACE
                    <span class="btn-subtitle">Cancel current mission</span>
                </button>
                <select class="safe-place-select" id="safe-place-select">
                    <option value="0">Home Base (0.0, 0.0)</option>
                    <option value="1">Charging Station (2.0, -1.0)</option>
                    <option value="2">Exit Point (-3.0, 5.0)</option>
                </select>
                <button class="action-btn btn-stay" onclick="stayStopd()">
                    ⏸️ STAY STOPPED
                    <span class="btn-subtitle">Keep robot stationary</span>
                </button>
            </div>
        </div>
    </div>

    <script>
        // ============================================
        // MANRIIX NAVIGATION GUIDANCE SYSTEM
        // Remote Visualizer - JavaScript
        // ============================================

        // Global state
        let ros = null;
        let rosConnected = false;
        let robotPose = { x: 0, y: 0, theta: 0, vx: 0, vz: 0 };
        let previousGoal = { x: 0, y: 0 };
        let clusters = [];
        let humans = [];
        let optimalPositions = [];
        let laserScan = [];
        let globalPath = [];
        let mapData = null;
        let mapInfo = null;
        let costmapData = null;
        let costmapInfo = null;
        let explorationGoal = null;
        let isStopped = false;
        let holdTimer = null;
        let holdProgress = 0;

        // Map visualization state
        let mapView = {
            scale: 50,
            offsetX: 0,
            offsetY: 0,
            isDragging: false,
            dragStart: { x: 0, y: 0 }
        };

        // Layer visibility
        let layers = {
            laser: true,
            path: true,
            costmap: true,
            frontiers: true,
            pois: true
        };

        // Camera topics - Direct SDK mode publishes images with bboxes pre-drawn
        // These topics come from direct_zed_detection_node.py
        const cameraTopics = {
            left: '/viz/camera/zedx_left/image_rect_color/compressed',
            front: '/viz/camera/zedx_front/image_rect_color/compressed',
            right: '/viz/camera/zedx_right/image_rect_color/compressed'
        };

        // Detection topics - Not used in DIRECT_SDK mode (bboxes are pre-drawn)
        // Kept for backwards compatibility with ROS_WRAPPER mode
        const detectionTopics = {
            left: '/zedx_left/zed_node/obj_det/objects',
            front: '/zedx_front/zed_node/obj_det/objects',
            right: '/zedx_right/zed_node/obj_det/objects'
        };

        // Detection storage
        let detections = { left: [], front: [], right: [] };

        // Photo session state
        let photoSession = {
            status: 'idle',
            assignedDuration: 0,
            startTime: null,
            remaining: 0
        };

        // Event log
        let events = [];

        // Recovery level names
        const recoveryLevelNames = [
            'NORMAL', 'WAIT_AND_MONITOR', 'SENSOR_RESET', 'ORIENTATION_SCANNING',
            'POSITION_RELOCATION', 'EXPLORATION_MODE', 'SYSTEM_DIAGNOSTIC',
            'SAFE_PARKING', 'EMERGENCY_STOP'
        ];

        // Safe places (from config)
        const safePlaces = ''' + json.dumps(CONFIG['safe_places']) + ''';

        // ============================================
        // PAGE NAVIGATION
        // ============================================

        function showPage(page) {
            document.querySelectorAll('.page').forEach(p => p.classList.remove('active'));
            document.querySelectorAll('.nav-tab').forEach(t => t.classList.remove('active'));
            document.getElementById('page-' + page).classList.add('active');
            event.target.classList.add('active');

            if (page === 'live') {
                resizeMapCanvas();
            }
        }

        // ============================================
        // ROSBRIDGE CONNECTION
        // ============================================

        function connectROS() {
            const wsUrl = 'ws://' + window.location.hostname + ':9090';
            ros = new ROSLIB.Ros({ url: wsUrl });

            ros.on('connection', () => {
                rosConnected = true;
                document.getElementById('ros-status-dot').classList.add('connected');
                document.getElementById('ros-status-text').textContent = 'Connected';
                addEvent('ROSBridge connected', 'success');
                subscribeTopics();
            });

            ros.on('error', (error) => {
                console.log('ROS error:', error);
                addEvent('ROSBridge error', 'error');
            });

            ros.on('close', () => {
                rosConnected = false;
                document.getElementById('ros-status-dot').classList.remove('connected');
                document.getElementById('ros-status-text').textContent = 'Disconnected';
                addEvent('ROSBridge disconnected', 'warning');
                setTimeout(connectROS, 3000);
            });
        }

        // ============================================
        // TOPIC SUBSCRIPTIONS
        // ============================================

        function subscribeTopics() {
            if (!rosConnected) return;

            // Robot pose (odometry)
            new ROSLIB.Topic({
                ros: ros,
                name: '/odometry/filtered',
                messageType: 'nav_msgs/Odometry',
                throttle_rate: 100
            }).subscribe(msg => {
                robotPose.x = msg.pose.pose.position.x;
                robotPose.y = msg.pose.pose.position.y;
                const q = msg.pose.pose.orientation;
                robotPose.theta = Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
                robotPose.vx = msg.twist.twist.linear.x;
                robotPose.vz = msg.twist.twist.angular.z;
                updateRobotDisplay();
            });

            // Map
            new ROSLIB.Topic({
                ros: ros,
                name: '/map',
                messageType: 'nav_msgs/OccupancyGrid',
                throttle_rate: 2000
            }).subscribe(msg => {
                mapInfo = msg.info;
                mapData = msg.data;
            });

            // Costmap
            new ROSLIB.Topic({
                ros: ros,
                name: '/global_costmap/costmap',
                messageType: 'nav_msgs/OccupancyGrid',
                throttle_rate: 1000
            }).subscribe(msg => {
                costmapInfo = msg.info;
                costmapData = msg.data;
            });

            // Laser scan
            new ROSLIB.Topic({
                ros: ros,
                name: '/scan',
                messageType: 'sensor_msgs/LaserScan',
                throttle_rate: 100
            }).subscribe(msg => {
                laserScan = [];
                for (let i = 0; i < msg.ranges.length; i++) {
                    const r = msg.ranges[i];
                    if (r >= msg.range_min && r <= msg.range_max) {
                        const angle = msg.angle_min + i * msg.angle_increment;
                        laserScan.push({ x: r * Math.cos(angle), y: r * Math.sin(angle) });
                    }
                }
            });

            // Global path
            new ROSLIB.Topic({
                ros: ros,
                name: '/plan',
                messageType: 'nav_msgs/Path'
            }).subscribe(msg => {
                globalPath = msg.poses.map(p => ({ x: p.pose.position.x, y: p.pose.position.y }));
                if (globalPath.length > 0) {
                    previousGoal = globalPath[globalPath.length - 1];
                }
            });

            // Human clusters (standard MarkerArray for web interface)
            new ROSLIB.Topic({
                ros: ros,
                name: '/web/clusters',
                messageType: 'visualization_msgs/MarkerArray',
                throttle_rate: 200
            }).subscribe(msg => {
                // Parse clusters from markers - CYLINDER type (3)
                const clusterMarkers = msg.markers.filter(m => m.action === 0 && m.type === 3);
                clusters = clusterMarkers.map(m => ({
                    id: m.id,
                    x: m.pose.position.x,
                    y: m.pose.position.y,
                    radius: m.scale.x / 2, // Marker scale is diameter
                    size: parseInt(m.text) || 1,
                    formation: m.ns || 'unknown',
                    stability: m.color.a || 1.0
                }));
                updateClusterDisplay();
            });

            // Individual humans (standard MarkerArray for web interface)
            new ROSLIB.Topic({
                ros: ros,
                name: '/web/humans',
                messageType: 'visualization_msgs/MarkerArray',
                throttle_rate: 100
            }).subscribe(msg => {
                // Parse humans from markers - CYLINDER type (3)
                humans = msg.markers.filter(m => m.action === 0 && m.type === 3).map(m => ({
                    id: m.id,
                    x: m.pose.position.x,
                    y: m.pose.position.y,
                    confidence: m.color.a || 1.0
                }));
            });

            // Optimal position (standard Marker for web interface)
            new ROSLIB.Topic({
                ros: ros,
                name: '/web/optimal_position',
                messageType: 'visualization_msgs/Marker',
                throttle_rate: 500
            }).subscribe(msg => {
                if (msg.action === 0) { // ADD action
                    optimalPositions = [{
                        x: msg.pose.position.x,
                        y: msg.pose.position.y,
                        orientation: 0,
                        score: msg.color.a || 1.0
                    }];
                }
            });

            // Exploration goal
            new ROSLIB.Topic({
                ros: ros,
                name: '/exploration/goal',
                messageType: 'geometry_msgs/PoseStamped'
            }).subscribe(msg => {
                explorationGoal = {
                    x: msg.pose.position.x,
                    y: msg.pose.position.y
                };
            });

            // Mission status
            new ROSLIB.Topic({
                ros: ros,
                name: '/mission/status',
                messageType: 'std_msgs/String',
                throttle_rate: 500
            }).subscribe(msg => {
                try {
                    const data = JSON.parse(msg.data);
                    updateMissionStatus(data);
                } catch (e) {
                    updateMissionState(msg.data);
                }
            });

            // Mission metrics
            new ROSLIB.Topic({
                ros: ros,
                name: '/mission/metrics',
                messageType: 'std_msgs/String',
                throttle_rate: 1000
            }).subscribe(msg => {
                try {
                    const data = JSON.parse(msg.data);
                    updateMissionMetrics(data);
                } catch (e) {}
            });

            // Photo status
            new ROSLIB.Topic({
                ros: ros,
                name: '/photo/intelligence/status',
                messageType: 'std_msgs/String',
                throttle_rate: 500
            }).subscribe(msg => {
                updatePhotoStatus(msg.data);
            });

            // Photo metrics
            new ROSLIB.Topic({
                ros: ros,
                name: '/photo/intelligence/metrics',
                messageType: 'std_msgs/String',
                throttle_rate: 500
            }).subscribe(msg => {
                try {
                    const data = JSON.parse(msg.data);
                    updatePhotoMetrics(data);
                } catch (e) {}
            });

            // Recovery status
            new ROSLIB.Topic({
                ros: ros,
                name: '/recovery/status',
                messageType: 'std_msgs/String',
                throttle_rate: 1000
            }).subscribe(msg => {
                try {
                    const data = JSON.parse(msg.data);
                    updateRecoveryStatus(data);
                } catch (e) {}
            });

            // System health
            new ROSLIB.Topic({
                ros: ros,
                name: '/recovery/system_health',
                messageType: 'std_msgs/String',
                throttle_rate: 2000
            }).subscribe(msg => {
                try {
                    const data = JSON.parse(msg.data);
                    updateSystemHealth(data);
                } catch (e) {}
            });

            // POI metrics
            new ROSLIB.Topic({
                ros: ros,
                name: '/poi_manager/metrics',
                messageType: 'std_msgs/String',
                throttle_rate: 1000
            }).subscribe(msg => {
                try {
                    const data = JSON.parse(msg.data);
                    updatePOIMetrics(data);
                } catch (e) {}
            });

            // All POIs
            new ROSLIB.Topic({
                ros: ros,
                name: '/poi_manager/all_pois',
                messageType: 'std_msgs/String',
                throttle_rate: 1000
            }).subscribe(msg => {
                try {
                    const data = JSON.parse(msg.data);
                    updateAllPOIs(data);
                } catch (e) {}
            });

            // Clustering FPS
            new ROSLIB.Topic({
                ros: ros,
                name: '/human_clustering/fps',
                messageType: 'std_msgs/Float32',
                throttle_rate: 1000
            }).subscribe(msg => {
                document.getElementById('perf-clustering').textContent = msg.data.toFixed(1) + ' Hz';
            });

            // Subscribe to cameras
            subscribeCameras();
            subscribeDetections();
        }

        // ============================================
        // CAMERA SUBSCRIPTIONS
        // ============================================

        function subscribeCameras() {
            ['left', 'front', 'right'].forEach(cam => {
                new ROSLIB.Topic({
                    ros: ros,
                    name: cameraTopics[cam],
                    messageType: 'sensor_msgs/CompressedImage',
                    throttle_rate: 66  // ~15 FPS for smoother video (was 100ms = 10 FPS)
                }).subscribe(msg => {
                    const img = document.getElementById('img-' + cam);
                    const placeholder = document.getElementById('placeholder-' + cam);
                    if (img && msg.data) {
                        img.src = 'data:image/jpeg;base64,' + msg.data;
                        img.style.display = 'block';
                        if (placeholder) placeholder.style.display = 'none';
                    }
                });
            });
        }

        function subscribeDetections() {
            // Subscribe to camera detections JSON from direct_zed_detection_node
            // Note: In DIRECT_SDK mode, bounding boxes are pre-drawn on images
            // This subscription is for updating detection count displays only
            new ROSLIB.Topic({
                ros: ros,
                name: '/web/camera_detections',
                messageType: 'std_msgs/String',
                throttle_rate: 100
            }).subscribe(msg => {
                try {
                    const data = JSON.parse(msg.data);
                    // Update detection count displays per camera
                    // Bounding boxes are pre-drawn by direct_zed_detection_node
                    Object.keys(data).forEach(cam => {
                        // Handle both 'front' and 'zedx_front' formats
                        const camKey = cam.replace('zedx_', '');
                        cameraDetections[camKey] = data[cam];

                        // Update detection count display
                        const countEl = document.getElementById('count-' + camKey);
                        if (countEl) {
                            countEl.textContent = data[cam].count + ' persons';
                        }
                    });
                } catch (e) {
                    console.error('Error parsing detections:', e);
                }
            });
        }

        function drawBoundingBoxes(cam) {
            const canvas = document.getElementById('bbox-' + cam);
            const img = document.getElementById('img-' + cam);
            if (!canvas || !img) return;

            const ctx = canvas.getContext('2d');

            // Get displayed image dimensions
            const displayWidth = img.clientWidth || img.naturalWidth || 640;
            const displayHeight = img.clientHeight || img.naturalHeight || 360;

            // Set canvas to match displayed image size
            canvas.width = displayWidth;
            canvas.height = displayHeight;
            ctx.clearRect(0, 0, canvas.width, canvas.height);

            // ZED grab_resolution setting determines BOTH image AND detection resolution
            // Current config: HD1080 (1920x1080) for YOLO custom detection
            // BBox coordinates match the grab resolution
            //
            // Detection resolution (where bbox coords come from):
            const detectionWidth = 1920;
            const detectionHeight = 1080;

            // Scale factors from detection resolution to displayed size
            const scaleX = displayWidth / detectionWidth;
            const scaleY = displayHeight / detectionHeight;

            // Debug: Log resolution info once per camera
            if (!window._resolutionLogged) window._resolutionLogged = {};
            if (!window._resolutionLogged[cam]) {
                console.log(`[${cam}] Detection res: ${detectionWidth}x${detectionHeight}, Display: ${displayWidth}x${displayHeight}, Scale: ${scaleX.toFixed(3)}x${scaleY.toFixed(3)}`);
                window._resolutionLogged[cam] = true;
            }

            detections[cam].forEach((det, idx) => {
                let x1, y1, x2, y2;

                // bbox is in [[x,y], [x,y], [x,y], [x,y]] format from JSON
                // Corner order: [top-left, top-right, bottom-right, bottom-left]
                if (Array.isArray(det.bbox) && det.bbox.length >= 4) {
                    const c0 = det.bbox[0]; // Top-left
                    const c2 = det.bbox[2]; // Bottom-right

                    // Handle [[x,y], ...] format (from our JSON publisher)
                    if (Array.isArray(c0) && c0.length >= 2) {
                        x1 = c0[0] ?? 0;
                        y1 = c0[1] ?? 0;
                        x2 = c2?.[0] ?? 0;
                        y2 = c2?.[1] ?? 0;
                    }
                    // Fallback for {kp: [x, y]} format (ZED SDK via ROSBridge)
                    else if (c0?.kp && Array.isArray(c0.kp)) {
                        x1 = c0.kp[0] ?? 0;
                        y1 = c0.kp[1] ?? 0;
                        x2 = c2?.kp?.[0] ?? 0;
                        y2 = c2?.kp?.[1] ?? 0;
                    }
                    // Fallback for {x, y} format
                    else if (c0?.x !== undefined) {
                        x1 = c0.x ?? 0;
                        y1 = c0.y ?? 0;
                        x2 = c2?.x ?? 0;
                        y2 = c2?.y ?? 0;
                    } else {
                        return; // Skip invalid bbox
                    }
                } else if (det.bbox?.corners && Array.isArray(det.bbox.corners)) {
                    // Format: {corners: [{kp: [x,y]}, ...]}
                    const corners = det.bbox.corners;
                    if (corners.length >= 4) {
                        x1 = corners[0]?.kp?.[0] ?? corners[0]?.[0] ?? 0;
                        y1 = corners[0]?.kp?.[1] ?? corners[0]?.[1] ?? 0;
                        x2 = corners[2]?.kp?.[0] ?? corners[2]?.[0] ?? 0;
                        y2 = corners[2]?.kp?.[1] ?? corners[2]?.[1] ?? 0;
                    } else {
                        return;
                    }
                } else {
                    return; // Skip invalid or empty bbox
                }

                // Validate raw coordinates (must have some size)
                if (x2 <= x1 || y2 <= y1) return;

                // Scale coordinates from source resolution to display size
                x1 = x1 * scaleX;
                y1 = y1 * scaleY;
                x2 = x2 * scaleX;
                y2 = y2 * scaleY;

                // Clamp to canvas bounds
                x1 = Math.max(0, Math.min(x1, displayWidth));
                y1 = Math.max(0, Math.min(y1, displayHeight));
                x2 = Math.max(0, Math.min(x2, displayWidth));
                y2 = Math.max(0, Math.min(y2, displayHeight));

                // Color based on tracking state (green for OK, yellow for searching, blue for other)
                const color = det.tracking_state === 1 ? '#3fb950' :
                              det.tracking_state === 2 ? '#f0c000' : '#58a6ff';

                ctx.strokeStyle = color;
                ctx.lineWidth = 2;
                ctx.strokeRect(x1, y1, x2 - x1, y2 - y1);

                // Semi-transparent fill
                const r = parseInt(color.slice(1, 3), 16);
                const g = parseInt(color.slice(3, 5), 16);
                const b = parseInt(color.slice(5, 7), 16);
                ctx.fillStyle = `rgba(${r}, ${g}, ${b}, 0.15)`;
                ctx.fillRect(x1, y1, x2 - x1, y2 - y1);

                // Draw label with confidence
                if (det.confidence > 0) {
                    ctx.fillStyle = color;
                    ctx.font = 'bold 11px sans-serif';
                    const labelY = y1 > 15 ? y1 - 4 : y1 + 14;
                    ctx.fillText(`${det.label} ${det.confidence.toFixed(0)}%`, x1 + 2, labelY);
                }
            });
        }

        // ============================================
        // UI UPDATE FUNCTIONS
        // ============================================

        function updateRobotDisplay() {
            document.getElementById('robot-x').textContent = robotPose.x.toFixed(2) + ' m';
            document.getElementById('robot-y').textContent = robotPose.y.toFixed(2) + ' m';
            document.getElementById('robot-theta').textContent = (robotPose.theta * 180 / Math.PI).toFixed(1) + '°';
            document.getElementById('robot-speed').textContent = robotPose.vx.toFixed(2) + ' m/s';
        }

        function updateClusterDisplay() {
            let totalHumans = 0;
            clusters.forEach(c => totalHumans += c.size);
            document.getElementById('humans-count').textContent = totalHumans;
            document.getElementById('clusters-count').textContent = clusters.length;
        }

        function updateMissionState(state) {
            const badge = document.getElementById('mission-state');
            badge.textContent = state.toUpperCase();
            badge.className = 'state-badge';

            const stateMap = {
                'idle': 'state-idle',
                'evaluating_target': 'state-evaluating',
                'positioning_optimal': 'state-positioning',
                'waiting_for_stability': 'state-stabilizing',
                'photo_sequence': 'state-capturing',
                'moving_to_next': 'state-moving',
                'recovery': 'state-recovery'
            };

            badge.classList.add(stateMap[state.toLowerCase()] || 'state-idle');
        }

        function updateMissionStatus(data) {
            if (data.current_state) updateMissionState(data.current_state);
            if (data.efficiency !== undefined) {
                document.getElementById('efficiency-value').textContent = Math.round(data.efficiency * 100);
            }
        }

        function updateMissionMetrics(data) {
            if (data.photos_completed !== undefined) {
                document.getElementById('metric-photos').textContent = data.photos_completed;
            }
            if (data.targets_evaluated !== undefined) {
                document.getElementById('metric-targets').textContent = data.targets_evaluated;
            }
            if (data.total_distance !== undefined) {
                document.getElementById('metric-distance').textContent = data.total_distance.toFixed(1);
            }
            if (data.efficiency !== undefined) {
                document.getElementById('metric-efficiency').textContent = Math.round(data.efficiency * 100) + '%';
                document.getElementById('efficiency-value').textContent = Math.round(data.efficiency * 100);
            }
        }

        function updatePhotoStatus(status) {
            photoSession.status = status.toLowerCase();
            const indicator = document.getElementById('photo-indicator');
            const text = document.getElementById('photo-status-text');

            if (status.toLowerCase() === 'active' || status.toLowerCase() === 'capturing') {
                indicator.classList.add('active');
                text.textContent = 'Capturing...';
                if (!photoSession.startTime) {
                    photoSession.startTime = Date.now();
                }
            } else {
                indicator.classList.remove('active');
                text.textContent = status;
                photoSession.startTime = null;
            }
        }

        function updatePhotoMetrics(data) {
            if (data.optimal_duration !== undefined) {
                photoSession.assignedDuration = data.optimal_duration;
                document.getElementById('photo-assigned').textContent = data.optimal_duration + 's';
            }
            if (data.group_size !== undefined) {
                document.getElementById('intel-group-size').textContent = data.group_size;
            }
            if (data.crowd_density !== undefined) {
                document.getElementById('intel-density').textContent = data.crowd_density.toFixed(1);
            }
            if (data.formation_stability !== undefined) {
                document.getElementById('intel-stability').textContent = Math.round(data.formation_stability * 100) + '%';
            }
            if (data.overall_photo_readiness !== undefined) {
                document.getElementById('intel-readiness').textContent = Math.round(data.overall_photo_readiness * 100) + '%';
            }
        }

        function updateRecoveryStatus(data) {
            const alert = document.getElementById('recovery-alert');
            const level = data.current_level || 0;

            if (level > 0) {
                alert.classList.add('active');
                document.getElementById('recovery-level').textContent = level;
                document.getElementById('recovery-name').textContent = recoveryLevelNames[level] || 'UNKNOWN';
                document.getElementById('recovery-attempts').textContent = data.recovery_attempts || 0;
            } else {
                alert.classList.remove('active');
            }

            // Update system page
            document.getElementById('sys-recovery-level').textContent = level + ' (' + (recoveryLevelNames[level] || 'UNKNOWN') + ')';

            // Update level boxes
            const boxes = document.querySelectorAll('.recovery-level-box');
            boxes.forEach((box, i) => {
                box.classList.remove('current', 'passed');
                if (i < level) box.classList.add('passed');
                if (i === level) box.classList.add('current');
            });

            if (data.total_recovery_sessions !== undefined) {
                document.getElementById('recovery-sessions').textContent = data.total_recovery_sessions;
            }
            if (data.successful_recoveries !== undefined) {
                document.getElementById('recovery-success').textContent = data.successful_recoveries;
            }
            if (data.avg_recovery_time !== undefined) {
                document.getElementById('recovery-avg-time').textContent = data.avg_recovery_time.toFixed(1) + 's';
            }
        }

        function updateSystemHealth(data) {
            if (data.cpu_usage !== undefined) {
                const cpu = data.cpu_usage;
                document.getElementById('cpu-value').textContent = cpu.toFixed(1) + '%';
                const cpuBar = document.getElementById('cpu-bar');
                cpuBar.style.width = cpu + '%';
                cpuBar.className = 'resource-fill' + (cpu > 80 ? ' danger' : cpu > 60 ? ' warning' : '');
            }
            if (data.memory_usage !== undefined) {
                const mem = data.memory_usage;
                document.getElementById('mem-value').textContent = mem.toFixed(1) + '%';
                const memBar = document.getElementById('mem-bar');
                memBar.style.width = mem + '%';
                memBar.className = 'resource-fill' + (mem > 80 ? ' danger' : mem > 60 ? ' warning' : '');
            }
            if (data.disk_usage !== undefined) {
                const disk = data.disk_usage;
                document.getElementById('disk-value').textContent = disk.toFixed(1) + '%';
                document.getElementById('disk-bar').style.width = disk + '%';
            }

            // Update component status list
            if (data.node_status) {
                updateComponentStatus(data.node_status);
            }
        }

        function updateComponentStatus(nodeStatus) {
            const componentList = document.getElementById('component-list');

            // Component name mapping (display name -> possible node names)
            const componentMap = {
                'ZED Fusion': ['zed_fusion_node', 'zed_fusion'],
                'Human Clustering': ['human_clustering_node', 'human_clustering'],
                'POI Manager': ['poi_manager'],
                'Mission Controller': ['mission_controller'],
                'Photo Intelligence': ['photo_intelligence_system', 'photo_intelligence'],
                'Recovery Manager': ['recovery_manager']
            };

            let html = '';
            for (const [displayName, nodeNames] of Object.entries(componentMap)) {
                let status = 'idle';
                let statusText = 'Idle';

                // Check if any matching node is in the status
                for (const nodeName of nodeNames) {
                    if (nodeStatus[nodeName]) {
                        status = nodeStatus[nodeName];
                        statusText = status.charAt(0).toUpperCase() + status.slice(1);
                        break;
                    }
                }

                // If not found in node_status, check if we're receiving data from it
                if (status === 'idle') {
                    // Mark as active if we're getting updates
                    if (displayName === 'ZED Fusion' && Object.keys(detections).some(k => detections[k].length > 0)) {
                        status = 'active';
                        statusText = 'Active';
                    }
                }

                html += `
                    <div class="component-item">
                        <span class="component-name">\u25CF ${displayName}</span>
                        <span class="component-status ${status}">${statusText}</span>
                    </div>
                `;
            }

            componentList.innerHTML = html;
        }

        function updatePOIMetrics(data) {
            // Update average priority score
            if (data.average_priority_score !== undefined) {
                const score = (data.average_priority_score * 100).toFixed(1);
                document.getElementById('avg-priority-score').textContent = score + '%';
            }

            // Update number of POIs
            if (data.num_pois !== undefined) {
                document.getElementById('num-pois').textContent = data.num_pois;
            }

            // The 10-factor bars show the weight distribution (static)
            // But we can update the formation distribution if available
            if (data.formations) {
                console.log('Formation distribution:', data.formations);
            }

            // Update accessibility/stability scores if available (for backward compatibility)
            if (data.average_accessibility_score !== undefined) {
                const val = Math.round(data.average_accessibility_score * 100);
                const el = document.getElementById('score-accessibility');
                if (el) el.style.width = val + '%';
            }
            if (data.average_stability_score !== undefined) {
                const val = Math.round(data.average_stability_score * 100);
                const el = document.getElementById('score-stability');
                if (el) el.style.width = val + '%';
            }
        }

        function updateAllPOIs(data) {
            const list = document.getElementById('poi-list');
            if (!data || data.length === 0) {
                list.innerHTML = '<div class="poi-item"><span>No POIs available</span></div>';
                return;
            }

            list.innerHTML = data.slice(0, 5).map((poi, i) => `
                <div class="poi-item">
                    <span class="poi-rank">#${i + 1}</span>
                    <span class="poi-position">(${poi.position ? poi.position[0].toFixed(1) : '--'}, ${poi.position ? poi.position[1].toFixed(1) : '--'})</span>
                    <span class="poi-score">${poi.priority_score ? poi.priority_score.toFixed(2) : '--'}</span>
                    <span class="poi-formation">${poi.formation_type || '--'}</span>
                </div>
            `).join('');
        }

        function addEvent(message, type = 'info') {
            const now = new Date();
            const time = now.toTimeString().split(' ')[0];
            events.unshift({ time, message, type });
            if (events.length > 50) events.pop();

            const log = document.getElementById('event-log');
            log.innerHTML = events.map(e => `
                <div class="event-item">
                    <span class="event-time">${e.time}</span>
                    <span class="event-msg ${e.type}">${e.message}</span>
                </div>
            `).join('');
        }

        // ============================================
        // MAP RENDERING
        // ============================================

        function initMap() {
            const canvas = document.getElementById('map-canvas');
            canvas.addEventListener('mousedown', onMapMouseDown);
            canvas.addEventListener('mousemove', onMapMouseMove);
            canvas.addEventListener('mouseup', () => mapView.isDragging = false);
            canvas.addEventListener('mouseleave', () => mapView.isDragging = false);
            canvas.addEventListener('wheel', onMapWheel);

            // Touch events
            canvas.addEventListener('touchstart', onMapTouchStart);
            canvas.addEventListener('touchmove', onMapTouchMove);
            canvas.addEventListener('touchend', () => mapView.isDragging = false);

            resizeMapCanvas();
            window.addEventListener('resize', resizeMapCanvas);

            requestAnimationFrame(renderMap);
        }

        function resizeMapCanvas() {
            const container = document.querySelector('.map-canvas-container');
            if (!container) return;
            const canvas = document.getElementById('map-canvas');
            canvas.width = container.clientWidth;
            canvas.height = container.clientHeight;
        }

        function worldToCanvas(wx, wy) {
            const canvas = document.getElementById('map-canvas');
            const cx = canvas.width / 2;
            const cy = canvas.height / 2;
            return {
                x: cx + (wx - mapView.offsetX) * mapView.scale,
                y: cy - (wy - mapView.offsetY) * mapView.scale
            };
        }

        function canvasToWorld(cx, cy) {
            const canvas = document.getElementById('map-canvas');
            const centerX = canvas.width / 2;
            const centerY = canvas.height / 2;
            return {
                x: mapView.offsetX + (cx - centerX) / mapView.scale,
                y: mapView.offsetY - (cy - centerY) / mapView.scale
            };
        }

        function renderMap() {
            const canvas = document.getElementById('map-canvas');
            const ctx = canvas.getContext('2d');
            const w = canvas.width;
            const h = canvas.height;

            // Clear
            ctx.fillStyle = '#0d1117';
            ctx.fillRect(0, 0, w, h);

            // Draw grid
            drawGrid(ctx, w, h);

            // Draw occupancy map
            if (mapData && mapInfo) {
                drawOccupancyMap(ctx, mapData, mapInfo);
            }

            // Draw costmap
            if (layers.costmap && costmapData && costmapInfo) {
                drawCostmap(ctx, costmapData, costmapInfo);
            }

            // Draw path
            if (layers.path && globalPath.length > 1) {
                drawPath(ctx, globalPath);
            }

            // Draw laser scan
            if (layers.laser && laserScan.length > 0) {
                drawLaserScan(ctx);
            }

            // Draw exploration frontier
            if (layers.frontiers && explorationGoal) {
                drawFrontier(ctx);
            }

            // Draw humans (blue color)
            humans.forEach(h => {
                const pos = worldToCanvas(h.x, h.y);
                ctx.fillStyle = '#58a6ff';  // Blue color for humans
                ctx.strokeStyle = '#ffffff';
                ctx.lineWidth = 2;
                ctx.beginPath();
                ctx.arc(pos.x, pos.y, 8, 0, Math.PI * 2);
                ctx.fill();
                ctx.stroke();
            });

            // Draw clusters
            clusters.forEach(c => {
                const pos = worldToCanvas(c.x, c.y);
                const radius = c.radius * mapView.scale;

                // Circle
                ctx.strokeStyle = '#39c5cf';
                ctx.lineWidth = 2;
                ctx.beginPath();
                ctx.arc(pos.x, pos.y, Math.max(radius, 15), 0, Math.PI * 2);
                ctx.stroke();

                // Fill
                ctx.fillStyle = 'rgba(57, 197, 207, 0.1)';
                ctx.fill();

                // Label
                ctx.fillStyle = '#39c5cf';
                ctx.font = 'bold 12px sans-serif';
                ctx.textAlign = 'center';
                ctx.fillText(c.size, pos.x, pos.y + 4);
            });

            // Draw POIs
            if (layers.pois) {
                optimalPositions.forEach(poi => {
                    const pos = worldToCanvas(poi.x, poi.y);
                    drawStar(ctx, pos.x, pos.y, 5, 12, 6, '#f0c000');
                });
            }

            // Draw robot
            drawRobot(ctx);

            requestAnimationFrame(renderMap);
        }

        function drawGrid(ctx, w, h) {
            ctx.strokeStyle = '#21262d';
            ctx.lineWidth = 1;

            const startWorld = canvasToWorld(0, h);
            const endWorld = canvasToWorld(w, 0);

            for (let wx = Math.floor(startWorld.x); wx <= Math.ceil(endWorld.x); wx++) {
                const pos = worldToCanvas(wx, 0);
                ctx.beginPath();
                ctx.moveTo(pos.x, 0);
                ctx.lineTo(pos.x, h);
                ctx.stroke();
            }

            for (let wy = Math.floor(startWorld.y); wy <= Math.ceil(endWorld.y); wy++) {
                const pos = worldToCanvas(0, wy);
                ctx.beginPath();
                ctx.moveTo(0, pos.y);
                ctx.lineTo(w, pos.y);
                ctx.stroke();
            }
        }

        function drawOccupancyMap(ctx, data, info) {
            const resolution = info.resolution;
            const originX = info.origin.position.x;
            const originY = info.origin.position.y;
            const width = info.width;
            const height = info.height;

            for (let y = 0; y < height; y += 2) {
                for (let x = 0; x < width; x += 2) {
                    const idx = y * width + x;
                    const val = data[idx];

                    if (val === -1) continue; // Unknown

                    const wx = originX + x * resolution;
                    const wy = originY + y * resolution;
                    const pos = worldToCanvas(wx, wy);
                    const size = Math.max(2, resolution * mapView.scale * 2);

                    if (val === 0) {
                        ctx.fillStyle = '#21262d'; // Free
                    } else {
                        ctx.fillStyle = '#8b949e'; // Occupied
                    }
                    ctx.fillRect(pos.x, pos.y - size, size, size);
                }
            }
        }

        function drawCostmap(ctx, data, info) {
            const resolution = info.resolution;
            const originX = info.origin.position.x;
            const originY = info.origin.position.y;
            const width = info.width;
            const height = info.height;

            for (let y = 0; y < height; y += 3) {
                for (let x = 0; x < width; x += 3) {
                    const idx = y * width + x;
                    const val = data[idx];

                    if (val < 30) continue;

                    const wx = originX + x * resolution;
                    const wy = originY + y * resolution;
                    const pos = worldToCanvas(wx, wy);
                    const size = Math.max(3, resolution * mapView.scale * 3);

                    const alpha = Math.min(val / 100, 1) * 0.4;
                    ctx.fillStyle = `rgba(240, 192, 0, ${alpha})`;
                    ctx.fillRect(pos.x, pos.y - size, size, size);
                }
            }
        }

        function drawPath(ctx, path) {
            if (path.length < 2) return;

            ctx.strokeStyle = '#f0c000';
            ctx.lineWidth = 2;
            ctx.beginPath();

            const start = worldToCanvas(path[0].x, path[0].y);
            ctx.moveTo(start.x, start.y);

            for (let i = 1; i < path.length; i++) {
                const pos = worldToCanvas(path[i].x, path[i].y);
                ctx.lineTo(pos.x, pos.y);
            }
            ctx.stroke();
        }

        function drawLaserScan(ctx) {
            ctx.fillStyle = '#f85149';
            const cosTheta = Math.cos(robotPose.theta + Math.PI);
            const sinTheta = Math.sin(robotPose.theta + Math.PI);

            laserScan.forEach(p => {
                const wx = robotPose.x + p.x * cosTheta - p.y * sinTheta;
                const wy = robotPose.y + p.x * sinTheta + p.y * cosTheta;
                const pos = worldToCanvas(wx, wy);
                ctx.fillRect(pos.x - 1, pos.y - 1, 2, 2);
            });
        }

        function drawFrontier(ctx) {
            if (!explorationGoal) return;
            const pos = worldToCanvas(explorationGoal.x, explorationGoal.y);

            ctx.strokeStyle = '#a371f7';
            ctx.lineWidth = 2;
            ctx.beginPath();
            ctx.moveTo(pos.x, pos.y - 10);
            ctx.lineTo(pos.x + 8, pos.y + 8);
            ctx.lineTo(pos.x - 8, pos.y + 8);
            ctx.closePath();
            ctx.stroke();
        }

        function drawRobot(ctx) {
            const pos = worldToCanvas(robotPose.x, robotPose.y);

            ctx.save();
            ctx.translate(pos.x, pos.y);
            ctx.rotate(-robotPose.theta);

            // Robot body
            ctx.fillStyle = '#3fb950';
            ctx.beginPath();
            ctx.moveTo(15, 0);
            ctx.lineTo(-10, -10);
            ctx.lineTo(-10, 10);
            ctx.closePath();
            ctx.fill();

            // Outline
            ctx.strokeStyle = '#ffffff';
            ctx.lineWidth = 2;
            ctx.stroke();

            ctx.restore();
        }

        function drawStar(ctx, cx, cy, spikes, outerRadius, innerRadius, color) {
            ctx.fillStyle = color;
            ctx.beginPath();

            let rot = Math.PI / 2 * 3;
            const step = Math.PI / spikes;

            ctx.moveTo(cx, cy - outerRadius);

            for (let i = 0; i < spikes; i++) {
                let x = cx + Math.cos(rot) * outerRadius;
                let y = cy + Math.sin(rot) * outerRadius;
                ctx.lineTo(x, y);
                rot += step;

                x = cx + Math.cos(rot) * innerRadius;
                y = cy + Math.sin(rot) * innerRadius;
                ctx.lineTo(x, y);
                rot += step;
            }

            ctx.lineTo(cx, cy - outerRadius);
            ctx.closePath();
            ctx.fill();
        }

        // Map interaction
        function onMapMouseDown(e) {
            mapView.isDragging = true;
            mapView.dragStart = { x: e.clientX, y: e.clientY };
        }

        function onMapMouseMove(e) {
            if (!mapView.isDragging) return;
            const dx = e.clientX - mapView.dragStart.x;
            const dy = e.clientY - mapView.dragStart.y;
            mapView.offsetX -= dx / mapView.scale;
            mapView.offsetY += dy / mapView.scale;
            mapView.dragStart = { x: e.clientX, y: e.clientY };
        }

        function onMapWheel(e) {
            e.preventDefault();
            const factor = e.deltaY > 0 ? 0.9 : 1.1;
            mapView.scale = Math.max(10, Math.min(200, mapView.scale * factor));
        }

        function onMapTouchStart(e) {
            if (e.touches.length === 1) {
                mapView.isDragging = true;
                mapView.dragStart = { x: e.touches[0].clientX, y: e.touches[0].clientY };
            }
        }

        function onMapTouchMove(e) {
            if (!mapView.isDragging || e.touches.length !== 1) return;
            const dx = e.touches[0].clientX - mapView.dragStart.x;
            const dy = e.touches[0].clientY - mapView.dragStart.y;
            mapView.offsetX -= dx / mapView.scale;
            mapView.offsetY += dy / mapView.scale;
            mapView.dragStart = { x: e.touches[0].clientX, y: e.touches[0].clientY };
        }

        function toggleLayer(layer) {
            layers[layer] = !layers[layer];
            document.getElementById('layer-' + layer).classList.toggle('active');
        }

        // ============================================
        // MAXIMIZE/MINIMIZE
        // ============================================

        let maximizedElement = null;

        function toggleMaximize(elementId) {
            const element = document.getElementById(elementId);
            if (!element) return;

            if (maximizedElement === element) {
                // Minimize
                element.classList.remove('maximized');
                maximizedElement = null;
                document.body.style.overflow = '';
            } else {
                // Minimize previous if any
                if (maximizedElement) {
                    maximizedElement.classList.remove('maximized');
                }
                // Maximize this one
                element.classList.add('maximized');
                maximizedElement = element;
                document.body.style.overflow = 'hidden';
            }

            // Resize map canvas if map was toggled
            if (elementId === 'map-panel') {
                setTimeout(resizeMapCanvas, 100);
            }
        }

        // ESC key to minimize
        document.addEventListener('keydown', (e) => {
            if (e.key === 'Escape' && maximizedElement) {
                maximizedElement.classList.remove('maximized');
                maximizedElement = null;
                document.body.style.overflow = '';
            }
        });

        // ============================================
        // EMERGENCY STOP
        // ============================================

        function emergencyStop() {
            if (!rosConnected) {
                alert('Not connected to ROSBridge');
                return;
            }

            // Immediately publish e_stop
            const eStopTopic = new ROSLIB.Topic({
                ros: ros,
                name: '/e_stop',
                messageType: 'std_msgs/Bool'
            });
            eStopTopic.publish(new ROSLIB.Message({ data: true }));

            // Call stop_motor service
            const stopService = new ROSLIB.Service({
                ros: ros,
                name: '/stop_motor',
                serviceType: 'std_srvs/Trigger'
            });
            stopService.callService(new ROSLIB.ServiceRequest({}), () => {});

            // Update UI
            isStopped = true;
            document.getElementById('stop-position').textContent =
                `x: ${robotPose.x.toFixed(2)}, y: ${robotPose.y.toFixed(2)}, θ: ${(robotPose.theta * 180 / Math.PI).toFixed(0)}°`;
            document.getElementById('stop-goal').textContent =
                `x: ${previousGoal.x.toFixed(2)}, y: ${previousGoal.y.toFixed(2)}`;
            document.getElementById('stop-overlay').classList.add('active');

            addEvent('EMERGENCY STOP activated', 'error');
        }

        function startHold(action) {
            if (holdTimer) return;
            holdProgress = 0;

            holdTimer = setInterval(() => {
                holdProgress += 5;
                document.getElementById('resume-progress').style.width = holdProgress + '%';

                if (holdProgress >= 100) {
                    cancelHold();
                    if (action === 'resume') {
                        resumePreviousGoal();
                    }
                }
            }, 100);
        }

        function cancelHold() {
            if (holdTimer) {
                clearInterval(holdTimer);
                holdTimer = null;
            }
            holdProgress = 0;
            document.getElementById('resume-progress').style.width = '0%';
        }

        function resumePreviousGoal() {
            // Unlock e_stop
            const eStopTopic = new ROSLIB.Topic({
                ros: ros,
                name: '/e_stop',
                messageType: 'std_msgs/Bool'
            });
            eStopTopic.publish(new ROSLIB.Message({ data: false }));

            // Start motor
            const startService = new ROSLIB.Service({
                ros: ros,
                name: '/start_motor',
                serviceType: 'std_srvs/Trigger'
            });
            startService.callService(new ROSLIB.ServiceRequest({}), () => {});

            // Re-send previous goal
            if (previousGoal.x !== 0 || previousGoal.y !== 0) {
                const goalTopic = new ROSLIB.Topic({
                    ros: ros,
                    name: '/goal_pose',
                    messageType: 'geometry_msgs/PoseStamped'
                });
                goalTopic.publish(new ROSLIB.Message({
                    header: { frame_id: 'map' },
                    pose: {
                        position: { x: previousGoal.x, y: previousGoal.y, z: 0 },
                        orientation: { x: 0, y: 0, z: 0, w: 1 }
                    }
                }));
            }

            isStopped = false;
            document.getElementById('stop-overlay').classList.remove('active');
            addEvent('Resumed previous goal', 'success');
        }

        function goToSafePlace() {
            const select = document.getElementById('safe-place-select');
            const idx = parseInt(select.value);
            const place = safePlaces[idx];

            // Unlock e_stop
            const eStopTopic = new ROSLIB.Topic({
                ros: ros,
                name: '/e_stop',
                messageType: 'std_msgs/Bool'
            });
            eStopTopic.publish(new ROSLIB.Message({ data: false }));

            // Start motor
            const startService = new ROSLIB.Service({
                ros: ros,
                name: '/start_motor',
                serviceType: 'std_srvs/Trigger'
            });
            startService.callService(new ROSLIB.ServiceRequest({}), () => {});

            // Send safe place goal
            const goalTopic = new ROSLIB.Topic({
                ros: ros,
                name: '/goal_pose',
                messageType: 'geometry_msgs/PoseStamped'
            });
            goalTopic.publish(new ROSLIB.Message({
                header: { frame_id: 'map' },
                pose: {
                    position: { x: place.x, y: place.y, z: 0 },
                    orientation: { x: 0, y: 0, z: 0, w: 1 }
                }
            }));

            isStopped = false;
            document.getElementById('stop-overlay').classList.remove('active');
            addEvent('Navigating to ' + place.name, 'success');
        }

        function stayStopd() {
            // Keep e_stop active, just close overlay
            isStopped = true;
            document.getElementById('stop-overlay').classList.remove('active');
            addEvent('Staying stopped - manual control only', 'warning');
        }

        // ============================================
        // PHOTO COUNTDOWN TIMER
        // ============================================

        function updatePhotoCountdown() {
            if (photoSession.status === 'active' && photoSession.startTime && photoSession.assignedDuration > 0) {
                const elapsed = (Date.now() - photoSession.startTime) / 1000;
                const remaining = Math.max(0, photoSession.assignedDuration - elapsed);

                document.getElementById('photo-remaining').textContent = Math.ceil(remaining);

                const progress = ((photoSession.assignedDuration - remaining) / photoSession.assignedDuration) * 100;
                document.getElementById('photo-progress').style.width = progress + '%';

                if (remaining <= 0) {
                    photoSession.startTime = null;
                }
            }
        }

        // ============================================
        // INITIALIZATION
        // ============================================

        document.addEventListener('DOMContentLoaded', () => {
            connectROS();
            initMap();

            // Photo countdown timer
            setInterval(updatePhotoCountdown, 100);

            // Populate safe places dropdown
            const select = document.getElementById('safe-place-select');
            select.innerHTML = safePlaces.map((p, i) =>
                `<option value="${i}">${p.name} (${p.x}, ${p.y})</option>`
            ).join('');
        });
    </script>
</body>
</html>
'''

# ============================================
# FLASK ROUTES
# ============================================

@app.route('/')
def index():
    return render_template_string(HTML_TEMPLATE)

@app.route('/config')
def get_config():
    return jsonify(CONFIG)

@app.route('/health')
def health():
    return jsonify({'status': 'ok', 'service': 'remote_viz'})

# ============================================
# MAIN ENTRY POINT
# ============================================

def main():
    print("\n" + "=" * 60)
    print("  MANRIIX NAVIGATION GUIDANCE SYSTEM")
    print("  Remote Visualizer")
    print("=" * 60)
    print(f"\n  Web Interface: http://0.0.0.0:{CONFIG['web_port']}")
    print(f"  ROSBridge:     ws://localhost:{CONFIG['rosbridge_port']}")
    print("\n  Pages:")
    print("    - LIVE: Camera feeds, map, robot state")
    print("    - ANALYTICS: Mission metrics, POI analysis")
    print("    - SYSTEM: Health, recovery, diagnostics")
    print("\n" + "=" * 60 + "\n")

    app.run(host='0.0.0.0', port=CONFIG['web_port'], debug=False, threaded=True)

if __name__ == '__main__':
    main()
