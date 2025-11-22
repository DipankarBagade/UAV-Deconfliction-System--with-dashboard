# backend/api/main.py
"""
FastAPI REST Service for UAV Strategic Deconfliction
FlytBase-style API endpoints
"""

from fastapi import FastAPI, HTTPException, BackgroundTasks
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, Field
from typing import List, Tuple, Optional, Dict
import time
from datetime import datetime

# Import core modules
import sys
from pathlib import Path
backend_path = Path(__file__).parent.parent
sys.path.insert(0, str(backend_path))

from core.models import Mission, ConflictReport, create_mission_from_waypoints
from core.conflict_detector import ConflictDetector, ConflictDetectorConfig
from simulation.drone_generator import DroneTrajectoryGenerator, DroneGeneratorConfig

# Initialize FastAPI app
app = FastAPI(
    title="UAV Strategic Deconfliction Service",
    description="FlytBase-style deconfliction API for pre-flight clearance",
    version="1.0.0"
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, specify actual origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Global state
conflict_detector: Optional[ConflictDetector] = None
simulated_missions: List[Mission] = []
system_stats = {
    "initialized": False,
    "num_simulated_drones": 0,
    "total_checks": 0,
    "initialization_time": None
}


# ==================== Pydantic Models ====================

class WaypointRequest(BaseModel):
    """Waypoint data from client"""
    x: float = Field(..., description="X coordinate (meters)")
    y: float = Field(..., description="Y coordinate (meters)")
    z: float = Field(..., description="Z coordinate / altitude (meters)")
    timestamp: float = Field(..., description="Time in seconds from mission start")


class MissionRequest(BaseModel):
    """Mission validation request"""
    mission_id: str = Field(..., description="Unique mission identifier")
    drone_id: str = Field(default="PRIMARY", description="Drone identifier")
    waypoints: List[WaypointRequest] = Field(..., min_length=2, description="List of waypoints")
    safety_buffer: float = Field(default=10.0, ge=1.0, le=100.0, description="Safety separation distance (meters)")


class ConflictResponse(BaseModel):
    """Individual conflict information"""
    conflict_id: str
    primary_mission_id: str
    conflicting_drone_id: str
    location: Dict[str, float]
    timestamp: float
    distance: float
    severity: str


class MissionValidationResponse(BaseModel):
    """Response for mission validation"""
    mission_id: str
    status: str
    conflicts: List[ConflictResponse]
    summary: Dict[str, int]
    safe_percentage: float
    execution_time: float
    message: str


class SystemInitRequest(BaseModel):
    """Request to initialize the system"""
    num_drones: int = Field(default=5000, ge=10, le=10000, description="Number of simulated drones")
    airspace_x: float = Field(default=10000, description="Airspace X dimension (meters)")
    airspace_y: float = Field(default=10000, description="Airspace Y dimension (meters)")
    airspace_z: float = Field(default=500, description="Airspace Z dimension (meters)")
    safety_buffer: float = Field(default=10.0, description="Default safety buffer (meters)")


class SystemStatusResponse(BaseModel):
    """System status information"""
    initialized: bool
    num_simulated_drones: int
    total_checks_performed: int
    initialization_time: Optional[str]
    uptime_seconds: Optional[float]


# ==================== API Endpoints ====================

@app.get("/")
async def root():
    """Root endpoint"""
    return {
        "service": "UAV Strategic Deconfliction Service",
        "version": "1.0.0",
        "status": "operational",
        "endpoints": {
            "status": "/api/v1/status",
            "initialize": "/api/v1/initialize",
            "check_mission": "/api/v1/check_mission",
            "statistics": "/api/v1/statistics"
        }
    }


@app.get("/api/v1/status", response_model=SystemStatusResponse)
async def get_status():
    """
    Get system status
    Similar to FlytBase's health check endpoint
    """
    uptime = None
    if system_stats["initialization_time"]:
        uptime = time.time() - system_stats["initialization_time"]
    
    return SystemStatusResponse(
        initialized=system_stats["initialized"],
        num_simulated_drones=system_stats["num_simulated_drones"],
        total_checks_performed=system_stats["total_checks"],
        initialization_time=datetime.fromtimestamp(system_stats["initialization_time"]).isoformat() if system_stats["initialization_time"] else None,
        uptime_seconds=uptime
    )


@app.post("/api/v1/initialize")
async def initialize_system(request: SystemInitRequest, background_tasks: BackgroundTasks):
    """
    Initialize the deconfliction system with simulated drones
    This must be called before checking missions
    
    FlytBase approach: Pre-load airspace with known flight plans
    """
    global conflict_detector, simulated_missions, system_stats
    
    print(f"\n{'='*60}")
    print(f"Initializing Deconfliction System")
    print(f"{'='*60}")
    
    start_time = time.time()
    
    # Define airspace bounds
    airspace_bounds = (
        (0, 0, 0),
        (request.airspace_x, request.airspace_y, request.airspace_z)
    )
    
    # Create conflict detector
    config = ConflictDetectorConfig(
        safety_buffer=request.safety_buffer,
        critical_distance=request.safety_buffer / 2,
        warning_distance=request.safety_buffer * 1.5
    )
    
    conflict_detector = ConflictDetector(
        airspace_bounds=airspace_bounds,
        config=config
    )
    
    # Generate simulated drones
    gen_config = DroneGeneratorConfig(
        num_drones=request.num_drones,
        airspace_bounds=airspace_bounds,
        safety_buffer=request.safety_buffer
    )
    
    generator = DroneTrajectoryGenerator(gen_config)
    simulated_missions = generator.generate_missions()
    
    # Index all simulated drones
    conflict_detector.index_simulation_drones(simulated_missions)
    
    # Update system stats
    init_time = time.time() - start_time
    system_stats.update({
        "initialized": True,
        "num_simulated_drones": len(simulated_missions),
        "total_checks": 0,
        "initialization_time": time.time()
    })
    
    print(f"\n✅ System initialized in {init_time:.2f}s")
    print(f"{'='*60}\n")
    
    return {
        "success": True,
        "message": f"System initialized with {len(simulated_missions)} simulated drones",
        "num_drones": len(simulated_missions),
        "initialization_time": init_time,
        "airspace": {
            "x": request.airspace_x,
            "y": request.airspace_y,
            "z": request.airspace_z
        }
    }


@app.post("/api/v1/check_mission", response_model=MissionValidationResponse)
async def check_mission(request: MissionRequest):
    """
    Strategic Deconfliction: Check if a mission is safe to execute
    
    This is the main FlytBase-style pre-flight clearance endpoint
    Similar to: POST /api/v1/mission/validate
    """
    global conflict_detector, system_stats
    
    # Check if system is initialized
    if not conflict_detector or not system_stats["initialized"]:
        raise HTTPException(
            status_code=503,
            detail="System not initialized. Call /api/v1/initialize first."
        )
    
    try:
        # Convert request to Mission object
        waypoint_coords = [
            (wp.x, wp.y, wp.z, wp.timestamp)
            for wp in request.waypoints
        ]
        
        mission = create_mission_from_waypoints(
            mission_id=request.mission_id,
            waypoint_coords=waypoint_coords,
            drone_id=request.drone_id,
            safety_buffer=request.safety_buffer
        )
        
        # Run conflict detection
        report = conflict_detector.check_mission(mission)
        
        # Update stats
        system_stats["total_checks"] += 1
        
        # Convert to response format
        conflicts = [
            ConflictResponse(
                conflict_id=c.conflict_id,
                primary_mission_id=c.primary_mission_id,
                conflicting_drone_id=c.conflicting_drone_id,
                location={
                    "x": float(c.location[0]),
                    "y": float(c.location[1]),
                    "z": float(c.location[2])
                },
                timestamp=c.timestamp,
                distance=c.distance,
                severity=c.severity
            )
            for c in report.conflicts
        ]
        
        # Create message
        if report.status == "CLEAR":
            message = "✅ Mission APPROVED for execution. No conflicts detected."
        else:
            message = f"⚠️ Mission REJECTED. {report.total_conflicts} conflict(s) detected ({report.critical_conflicts} critical)."
        
        return MissionValidationResponse(
            mission_id=report.mission_id,
            status=report.status,
            conflicts=conflicts,
            summary={
                "total_conflicts": report.total_conflicts,
                "critical_conflicts": report.critical_conflicts,
                "warning_conflicts": report.warning_conflicts
            },
            safe_percentage=report.safe_percentage,
            execution_time=report.execution_time,
            message=message
        )
    
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing mission: {str(e)}")


@app.get("/api/v1/statistics")
async def get_statistics():
    """
    Get detailed system statistics
    FlytBase-style analytics endpoint
    """
    if not conflict_detector:
        return {"error": "System not initialized"}
    
    detector_stats = conflict_detector.get_statistics()
    
    return {
        "system": system_stats,
        "detector": detector_stats,
        "airspace": {
            "grid_cells": detector_stats["spatial_index_stats"]["grid_stats"]["total_cells"],
            "occupied_cells": detector_stats["spatial_index_stats"]["grid_stats"]["occupied_cells"],
            "avg_drones_per_cell": detector_stats["spatial_index_stats"]["grid_stats"]["avg_drones_per_cell"]
        }
    }


@app.get("/api/v1/simulated_drones")
async def get_simulated_drones(limit: int = 10):
    """
    Get list of simulated drones (for debugging/visualization)
    """
    if not simulated_missions:
        return {"error": "No simulated drones available"}
    
    drones = []
    for mission in simulated_missions[:limit]:
        drones.append({
            "mission_id": mission.mission_id,
            "drone_id": mission.drone_id,
            "waypoints": len(mission.waypoints),
            "duration": mission.duration,
            "time_window": mission.time_window
        })
    
    return {
        "total_drones": len(simulated_missions),
        "showing": len(drones),
        "drones": drones
    }


# ==================== Health & Monitoring ====================

@app.get("/health")
async def health_check():
    """Health check endpoint for monitoring"""
    return {
        "status": "healthy",
        "timestamp": datetime.utcnow().isoformat(),
        "system_initialized": system_stats["initialized"]
    }

@app.get("/api/v1/visualization_data")
async def get_visualization_data(num_drones: int = 20):
    """
    Get trajectory data for visualization
    Returns waypoints for a sample of drones
    """
    if not simulated_missions:
        raise HTTPException(status_code=503, detail="No simulated drones available")
    
    # Get a random sample of drones
    import random
    sample_missions = random.sample(simulated_missions, min(num_drones, len(simulated_missions)))
    
    visualization_data = []
    
    for mission in sample_missions:
        waypoints = [
            {
                "x": float(wp.x),
                "y": float(wp.y),
                "z": float(wp.z),
                "timestamp": float(wp.timestamp)
            }
            for wp in mission.waypoints
        ]
        
        visualization_data.append({
            "drone_id": mission.drone_id,
            "mission_id": mission.mission_id,
            "waypoints": waypoints
        })
    
    return {
        "total_drones": len(simulated_missions),
        "returned_drones": len(visualization_data),
        "drones": visualization_data
    }

if __name__ == "__main__":
    import uvicorn
    
    print("\n" + "="*60)
    print("UAV Strategic Deconfliction Service")
    print("FlytBase-style REST API")
    print("="*60)
    print("\nStarting server on http://localhost:8000")
    print("API Documentation: http://localhost:8000/docs")
    print("="*60 + "\n")
    
    uvicorn.run(app, host="0.0.0.0", port=8000)