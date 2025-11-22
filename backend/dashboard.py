# backend/dashboard.py
"""
Streamlit Dashboard for UAV Strategic Deconfliction System
Interactive visualization and mission testing with 3D animation
"""

import streamlit as st
import plotly.graph_objects as go
import plotly.express as px
import numpy as np
import pandas as pd
import requests
import time
from typing import List, Dict

# Page config
st.set_page_config(
    page_title="UAV Deconfliction System",
    page_icon="🚁",
    layout="wide",
    initial_sidebar_state="expanded"
)

# API base URL
API_BASE = "http://localhost:8000"

# Custom CSS
st.markdown("""
<style>
    .main-header {
        font-size: 3rem;
        font-weight: bold;
        color: #1f77b4;
        text-align: center;
        margin-bottom: 2rem;
    }
    .status-clear {
        color: #28a745;
        font-weight: bold;
        font-size: 1.5rem;
    }
    .status-conflict {
        color: #dc3545;
        font-weight: bold;
        font-size: 1.5rem;
    }
</style>
""", unsafe_allow_html=True)

# Title
st.markdown('<div class="main-header">🚁 UAV Strategic Deconfliction System</div>', unsafe_allow_html=True)
st.markdown("---")

# Initialize session state
if 'system_initialized' not in st.session_state:
    st.session_state.system_initialized = False
if 'last_report' not in st.session_state:
    st.session_state.last_report = None
if 'check_history' not in st.session_state:
    st.session_state.check_history = []
if 'current_time' not in st.session_state:
    st.session_state.current_time = 0

# Sidebar - System Control
with st.sidebar:
    st.header("🎛️ System Control")
    
    # Check system status
    try:
        status_response = requests.get(f"{API_BASE}/api/v1/status")
        if status_response.status_code == 200:
            status = status_response.json()
            st.session_state.system_initialized = status['initialized']
            
            if status['initialized']:
                st.success("✅ System Online")
                st.metric("Simulated Drones", status['num_simulated_drones'])
                st.metric("Total Checks", status['total_checks_performed'])
                if status['uptime_seconds']:
                    st.metric("Uptime", f"{status['uptime_seconds']:.0f}s")
            else:
                st.warning("⚠️ System Not Initialized")
    except:
        st.error("❌ API Server Offline")
        st.info("Run: `python backend/api/main.py`")
    
    st.markdown("---")
    
    # System Initialization
    st.subheader("Initialize System")
    
    num_drones = st.number_input(
        "Number of Drones",
        min_value=100,
        max_value=10000,
        value=5000,
        step=100,
        help="Number of simulated drones in airspace"
    )
    
    col1, col2 = st.columns(2)
    with col1:
        airspace_x = st.number_input("Airspace X (m)", value=10000, step=1000)
    with col2:
        airspace_y = st.number_input("Airspace Y (m)", value=10000, step=1000)
    
    airspace_z = st.number_input("Airspace Z (m)", value=500, step=50)
    safety_buffer = st.slider("Safety Buffer (m)", 5.0, 50.0, 10.0, 5.0)
    
    if st.button("🚀 Initialize System", type="primary", use_container_width=True):
        with st.spinner(f"Initializing with {num_drones} drones..."):
            try:
                init_data = {
                    "num_drones": num_drones,
                    "airspace_x": airspace_x,
                    "airspace_y": airspace_y,
                    "airspace_z": airspace_z,
                    "safety_buffer": safety_buffer
                }
                
                response = requests.post(
                    f"{API_BASE}/api/v1/initialize",
                    json=init_data
                )
                
                if response.status_code == 200:
                    result = response.json()
                    st.success(f"✅ {result['message']}")
                    st.info(f"⏱️ Initialized in {result['initialization_time']:.2f}s")
                    st.session_state.system_initialized = True
                    st.rerun()
                else:
                    st.error(f"Error: {response.text}")
            except Exception as e:
                st.error(f"Connection error: {str(e)}")

# Main content area
if not st.session_state.system_initialized:
    st.info("👈 Please initialize the system using the sidebar controls")
    st.stop()

# Tabs
tab1, tab2, tab3, tab4 = st.tabs([
    "🎯 Mission Check",
    "📊 System Statistics",
    "🗺️ 3D Visualization",
    "📜 Check History"
])

# TAB 1: Mission Check
with tab1:
    st.header("Mission Validation")
    
    col1, col2 = st.columns([2, 1])
    
    with col1:
        st.subheader("Define Mission Waypoints")
        
        mission_id = st.text_input("Mission ID", value=f"MISSION_{int(time.time())}")
        drone_id = st.text_input("Drone ID", value="UAV_PRIMARY")
        mission_safety = st.slider("Safety Buffer (m)", 5.0, 100.0, 50.0, 5.0)
        
        input_method = st.radio("Input Method", ["Manual Entry", "Quick Presets"])
        
        waypoints_data = []
        
        if input_method == "Manual Entry":
            num_waypoints = st.number_input("Number of Waypoints", 2, 10, 3)
            
            st.markdown("#### Waypoint Coordinates")
            
            for i in range(num_waypoints):
                cols = st.columns(4)
                with cols[0]:
                    x = st.number_input(f"WP{i+1} X", value=float(i * 2000), key=f"x_{i}")
                with cols[1]:
                    y = st.number_input(f"WP{i+1} Y", value=float(i * 2000), key=f"y_{i}")
                with cols[2]:
                    z = st.number_input(f"WP{i+1} Z", value=100.0 + i * 20, key=f"z_{i}")
                with cols[3]:
                    t = st.number_input(f"WP{i+1} Time", value=float(i * 50), key=f"t_{i}")
                
                waypoints_data.append({"x": x, "y": y, "z": z, "timestamp": t})
        
        else:
            preset = st.selectbox("Select Preset", [
                "Short Delivery (Clear)",
                "Cross-City Flight (Potential Conflicts)",
                "High-Traffic Zone (High Conflicts)",
                "Low Altitude Patrol"
            ])
            
            if preset == "Short Delivery (Clear)":
                waypoints_data = [
                    {"x": 1000, "y": 1000, "z": 100, "timestamp": 0},
                    {"x": 2000, "y": 2000, "z": 150, "timestamp": 50}
                ]
            elif preset == "Cross-City Flight (Potential Conflicts)":
                waypoints_data = [
                    {"x": 1000, "y": 1000, "z": 100, "timestamp": 0},
                    {"x": 5000, "y": 5000, "z": 200, "timestamp": 300},
                    {"x": 9000, "y": 9000, "z": 100, "timestamp": 600}
                ]
            elif preset == "High-Traffic Zone (High Conflicts)":
                waypoints_data = [
                    {"x": 2000, "y": 2000, "z": 150, "timestamp": 0},
                    {"x": 5000, "y": 5000, "z": 200, "timestamp": 200},
                    {"x": 8000, "y": 8000, "z": 150, "timestamp": 400},
                    {"x": 5000, "y": 5000, "z": 180, "timestamp": 600}
                ]
            else:
                waypoints_data = [
                    {"x": 3000, "y": 3000, "z": 50, "timestamp": 0},
                    {"x": 4000, "y": 3000, "z": 50, "timestamp": 100},
                    {"x": 4000, "y": 4000, "z": 50, "timestamp": 200},
                    {"x": 3000, "y": 4000, "z": 50, "timestamp": 300},
                    {"x": 3000, "y": 3000, "z": 50, "timestamp": 400}
                ]
        
        if waypoints_data:
            st.markdown("#### Waypoint Summary")
            df_waypoints = pd.DataFrame(waypoints_data)
            st.dataframe(df_waypoints, use_container_width=True)
        
        if st.button("🔍 Check Mission for Conflicts", type="primary", use_container_width=True):
            if len(waypoints_data) < 2:
                st.error("Mission must have at least 2 waypoints")
            else:
                with st.spinner("Checking for conflicts..."):
                    try:
                        mission_data = {
                            "mission_id": mission_id,
                            "drone_id": drone_id,
                            "waypoints": waypoints_data,
                            "safety_buffer": mission_safety
                        }
                        
                        response = requests.post(
                            f"{API_BASE}/api/v1/check_mission",
                            json=mission_data
                        )
                        
                        if response.status_code == 200:
                            report = response.json()
                            st.session_state.last_report = report
                            st.session_state.check_history.append({
                                "mission_id": mission_id,
                                "timestamp": time.time(),
                                "status": report['status'],
                                "conflicts": report['summary']['total_conflicts']
                            })
                            st.rerun()
                        else:
                            st.error(f"Error: {response.text}")
                    except Exception as e:
                        st.error(f"Connection error: {str(e)}")
    
    with col2:
        st.subheader("Mission Report")
        
        if st.session_state.last_report:
            report = st.session_state.last_report
            
            if report['status'] == 'CLEAR':
                st.markdown(f'<div class="status-clear">✅ {report["status"]}</div>', unsafe_allow_html=True)
                st.success(report['message'])
            else:
                st.markdown(f'<div class="status-conflict">⚠️ {report["status"]}</div>', unsafe_allow_html=True)
                st.error(report['message'])
            
            col_a, col_b = st.columns(2)
            with col_a:
                st.metric("Total Conflicts", report['summary']['total_conflicts'])
                st.metric("Critical", report['summary']['critical_conflicts'])
            with col_b:
                st.metric("Safe %", f"{report['safe_percentage']:.1f}%")
                st.metric("Check Time", f"{report['execution_time']:.3f}s")
            
            if report['conflicts']:
                st.markdown("#### Conflict Details")
                
                conflicts_df = pd.DataFrame([
                    {
                        "Drone": c['conflicting_drone_id'],
                        "Time (s)": f"{c['timestamp']:.1f}",
                        "Distance (m)": f"{c['distance']:.2f}",
                        "Severity": c['severity'],
                        "X": f"{c['location']['x']:.0f}",
                        "Y": f"{c['location']['y']:.0f}",
                        "Z": f"{c['location']['z']:.0f}"
                    }
                    for c in report['conflicts'][:10]
                ])
                
                st.dataframe(conflicts_df, use_container_width=True)
                
                if len(report['conflicts']) > 10:
                    st.info(f"Showing 10 of {len(report['conflicts'])} conflicts")
        else:
            st.info("No mission checked yet.")

# TAB 2: Statistics
with tab2:
    st.header("System Statistics")
    
    try:
        stats_response = requests.get(f"{API_BASE}/api/v1/statistics")
        if stats_response.status_code == 200:
            stats = stats_response.json()
            
            col1, col2, col3, col4 = st.columns(4)
            
            with col1:
                st.metric("Indexed Drones", stats['system']['num_simulated_drones'])
            with col2:
                st.metric("Total Checks", stats['system']['total_checks'])
            with col3:
                st.metric("Grid Cells", stats['airspace']['grid_cells'])
            with col4:
                st.metric("Occupied Cells", stats['airspace']['occupied_cells'])
            
            st.subheader("Airspace Utilization")
            
            col1, col2 = st.columns(2)
            
            with col1:
                occupied = stats['airspace']['occupied_cells']
                empty = stats['airspace']['grid_cells'] - occupied
                
                fig_pie = go.Figure(data=[go.Pie(
                    labels=['Occupied', 'Empty'],
                    values=[occupied, empty],
                    hole=0.4,
                    marker_colors=['#1f77b4', '#e0e0e0']
                )])
                fig_pie.update_layout(title="Grid Cell Occupancy", height=300)
                st.plotly_chart(fig_pie, use_container_width=True)
            
            with col2:
                st.metric(
                    "Avg Drones per Cell",
                    f"{stats['airspace']['avg_drones_per_cell']:.2f}"
                )
                
                efficiency = (occupied / stats['airspace']['grid_cells']) * 100
                st.metric("Cell Efficiency", f"{efficiency:.1f}%")
    
    except Exception as e:
        st.error(f"Could not fetch statistics: {str(e)}")

# ==================== TAB 3: 3D Visualization ====================
with tab3:
    st.header("3D Airspace Visualization with Animation")
    
    # Visualization controls
    col1, col2, col3 = st.columns(3)
    
    with col1:
        show_conflicts = st.checkbox("Show Conflict Points", value=True)
    with col2:
        show_sample_drones = st.checkbox("Show Sample Drones", value=True)
    with col3:
        show_trails = st.checkbox("Show Drone Trails", value=True)
    
    num_sample_drones = st.slider("Number of Sample Drones", 5, 50, 20)
    
    # Time control
    st.markdown("### ⏱️ Time Control")
    max_time = 600
    current_time = st.slider("Current Time (seconds)", 0.0, float(max_time), 0.0, 10.0)
    
    col1, col2, col3, col4 = st.columns(4)
    with col1:
        if st.button("⏮️ Reset"):
            st.rerun()
    with col2:
        st.write("⏸️ Pause")
    with col3:
        st.write("▶️ Play")
    with col4:
        if st.button("🔄 Refresh"):
            st.rerun()
    
    # Create 3D plot
    fig = go.Figure()
    
    airspace_x, airspace_y, airspace_z = 10000, 10000, 500
    
    # Ground plane
    xx, yy = np.meshgrid([0, airspace_x], [0, airspace_y])
    zz = np.zeros_like(xx)
    fig.add_trace(go.Surface(
        x=xx, y=yy, z=zz,
        colorscale=[[0, 'lightgray'], [1, 'lightgray']],
        showscale=False,
        opacity=0.2,
        name='Ground',
        hoverinfo='skip'
    ))
    
    # Airspace boundaries
    boundary = [
        [0,0,0], [airspace_x,0,0], [airspace_x,airspace_y,0], [0,airspace_y,0], [0,0,0],
        [0,0,airspace_z], [airspace_x,0,airspace_z], [airspace_x,airspace_y,airspace_z],
        [0,airspace_y,airspace_z], [0,0,airspace_z]
    ]
    bx, by, bz = zip(*boundary)
    fig.add_trace(go.Scatter3d(x=bx, y=by, z=bz, mode='lines',
                               line=dict(color='gray', width=3), name='Airspace'))
    
    # Vertical edges
    for x, y in [(airspace_x,0), (airspace_x,airspace_y), (0,airspace_y)]:
        fig.add_trace(go.Scatter3d(x=[x,x], y=[y,y], z=[0,airspace_z],
                                   mode='lines', line=dict(color='gray', width=1, dash='dot'),
                                   showlegend=False))
    
    active_drones = 0
    
    # Get and display drone trajectories
    if show_sample_drones:
        try:
            viz_response = requests.get(f"{API_BASE}/api/v1/visualization_data",
                                       params={"num_drones": num_sample_drones})
            
            if viz_response.status_code == 200:
                drones = viz_response.json()['drones']
                
                for drone in drones:
                    wps = drone['waypoints']
                    x = [w['x'] for w in wps]
                    y = [w['y'] for w in wps]
                    z = [w['z'] for w in wps]
                    t = [w['timestamp'] for w in wps]
                    
                    # Random color
                    r, g, b = np.random.randint(50,200,3)
                    color_line = f'rgba({r},{g},{b},0.3)'
                    color_drone = f'rgba({r},{g},{b},1.0)'
                    
                    # Trail
                    if show_trails:
                        fig.add_trace(go.Scatter3d(x=x, y=y, z=z, mode='lines',
                                                  line=dict(color=color_line, width=2),
                                                  showlegend=False, hoverinfo='skip'))
                    
                    # Current position at current_time
                    if t[0] <= current_time <= t[-1]:
                        for j in range(len(t)-1):
                            if t[j] <= current_time <= t[j+1]:
                                alpha = (current_time - t[j]) / (t[j+1] - t[j])
                                x_now = x[j] + alpha * (x[j+1] - x[j])
                                y_now = y[j] + alpha * (y[j+1] - y[j])
                                z_now = z[j] + alpha * (z[j+1] - z[j])
                                
                                fig.add_trace(go.Scatter3d(
                                    x=[x_now], y=[y_now], z=[z_now],
                                    mode='markers',
                                    marker=dict(size=12, color=color_drone, symbol='circle',
                                              line=dict(color='white', width=2)),
                                    showlegend=False,
                                    hovertext=f"{drone['drone_id']}<br>t={current_time:.1f}s"
                                ))
                                active_drones += 1
                                break
                    
                    # Start/End markers
                    fig.add_trace(go.Scatter3d(x=[x[0]], y=[y[0]], z=[z[0]],
                                              mode='markers', marker=dict(size=8, color='green', symbol='diamond'),
                                              showlegend=False))
                    fig.add_trace(go.Scatter3d(x=[x[-1]], y=[y[-1]], z=[z[-1]],
                                              mode='markers', marker=dict(size=8, color='red', symbol='diamond'),
                                              showlegend=False))
                
                st.success(f"✅ {active_drones} drones active at t={current_time:.1f}s")
        except Exception as e:
            st.error(f"Error loading drones: {e}")
    
  # Show conflicts - IMPROVED VERSION
    if show_conflicts and st.session_state.last_report:
        report = st.session_state.last_report
        conflicts = report.get('conflicts', [])
        
        if conflicts:
            # Debug info
            st.info(f"📊 Total conflicts in report: {len(conflicts)}")
            
            conflict_times = [c['timestamp'] for c in conflicts]
            st.write(f"⏱️ Conflict time range: {min(conflict_times):.1f}s to {max(conflict_times):.1f}s")
            
            # Option to show all or filter by time
            show_all = st.checkbox("Show ALL conflicts (ignore time filter)", value=False)
            
            if show_all:
                visible = conflicts
                st.success(f"✅ Showing all {len(visible)} conflicts")
            else:
                time_window = 50
                visible = [c for c in conflicts if abs(c['timestamp'] - current_time) <= time_window]
                st.write(f"🔍 At t={current_time:.1f}s: {len(visible)} conflicts in ±{time_window}s window")
            
            if visible:
                # Extract all conflict locations
                conflict_x = [c['location']['x'] for c in visible]
                conflict_y = [c['location']['y'] for c in visible]
                conflict_z = [c['location']['z'] for c in visible]
                
                # Create hover text
                hover_texts = [
                    f"<b>⚠️ CONFLICT</b><br>" +
                    f"Time: {c['timestamp']:.1f}s<br>" +
                    f"Distance: {c['distance']:.2f}m<br>" +
                    f"Severity: {c['severity']}<br>" +
                    f"Conflicting Drone: {c['conflicting_drone_id']}<br>" +
                    f"Location: ({c['location']['x']:.0f}, {c['location']['y']:.0f}, {c['location']['z']:.0f})"
                    for c in visible
                ]
                
                # Plot ALL conflicts as large red X marks
                fig.add_trace(go.Scatter3d(
                    x=conflict_x,
                    y=conflict_y,
                    z=conflict_z,
                    mode='markers+text',
                    marker=dict(
                        size=10,  # Larger size
                        color='red',
                        symbol='x',
                        line=dict(color='darkred', width=6),
                        opacity=1.0
                    ),
                    text=['❌'] * len(visible),
                    textfont=dict(size=10, color='red', family='Arial Black'),
                    textposition='middle center',
                    name=f'⚠️ Conflicts ({len(visible)})',
                    hovertext=hover_texts,
                    hoverinfo='text',
                    showlegend=True
                ))
                
                st.error(f"⚠️ {len(visible)} CONFLICTS DISPLAYED AS RED X MARKS")
                
                # Show conflict details in expandable section
                with st.expander("📋 Conflict Details", expanded=False):
                    conflict_df = pd.DataFrame([
                        {
                            "Time (s)": c['timestamp'],
                            "X": c['location']['x'],
                            "Y": c['location']['y'],
                            "Z": c['location']['z'],
                            "Distance (m)": c['distance'],
                            "Severity": c['severity'],
                            "Drone ID": c['conflicting_drone_id']
                        }
                        for c in visible
                    ])
                    st.dataframe(conflict_df, use_container_width=True)
            else:
                nearest_time = min(conflict_times, key=lambda t: abs(t - current_time))
                st.info(f"ℹ️ No conflicts at t={current_time:.1f}s. Nearest conflict at t={nearest_time:.1f}s")
                st.write("💡 Try: Enable 'Show ALL conflicts' or move slider to conflict times")
    # Layout
    fig.update_layout(
        title=f"3D Airspace - Time: {current_time:.1f}s / {max_time}s",
        scene=dict(
            xaxis=dict(title='X (m)', range=[0,airspace_x], showgrid=True),
            yaxis=dict(title='Y (m)', range=[0,airspace_y], showgrid=True),
            zaxis=dict(title='Alt (m)', range=[0,airspace_z], showgrid=True),
            aspectmode='manual',
            aspectratio=dict(x=2, y=2, z=0.5),
            camera=dict(eye=dict(x=1.5, y=1.5, z=1.0))
        ),
        height=700,
        showlegend=True
    )
    
    st.plotly_chart(fig, use_container_width=True)
    
    with st.expander("ℹ️ How to Use"):
        st.markdown("""
        **Controls:**
        - 🎚️ Move time slider to animate
        - 🖱️ Click+drag to rotate
        - 🔍 Scroll to zoom
        
        **Legend:**
        - 🔵 Colored spheres = Active drones
        - 🟢 Green = Start
        - 🔴 Red = End
        - ❌ Red X = Conflicts
        """)

# TAB 4: History
with tab4:
    st.header("Mission Check History")
    
    if st.session_state.check_history:
        history_df = pd.DataFrame(st.session_state.check_history)
        history_df['time'] = pd.to_datetime(history_df['timestamp'], unit='s')
        
        st.dataframe(
            history_df[['mission_id', 'time', 'status', 'conflicts']],
            use_container_width=True
        )
        
        if len(history_df) > 1:
            fig = px.line(
                history_df,
                x='time',
                y='conflicts',
                markers=True,
                title="Conflicts Over Time"
            )
            st.plotly_chart(fig, use_container_width=True)
        
        st.subheader("Summary Statistics")
        col1, col2, col3, col4 = st.columns(4)
        
        with col1:
            st.metric("Total Checked", len(history_df))
        with col2:
            clear = len(history_df[history_df['status'] == 'CLEAR'])
            st.metric("Clear", clear)
        with col3:
            conflict = len(history_df[history_df['status'] == 'CONFLICT'])
            st.metric("Conflicts", conflict)
        with col4:
            total = history_df['conflicts'].sum()
            st.metric("Total Issues", int(total))
        
        if st.button("🗑️ Clear History"):
            st.session_state.check_history = []
            st.rerun()
    else:
        st.info("No checks yet")

st.markdown("---")
st.markdown("UAV Strategic Deconfliction System ")