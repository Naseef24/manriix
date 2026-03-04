/**
 * Manriix Robot Control - JavaScript Application
 * With Launch Orchestration
 */

// ==================== STATE ====================
const state = {
    currentStep: 'step-mode',
    selectedMode: null,
    selectedMap: null,
    mapSource: 'existing',
    waypoints: [],
    waitMode: 'time_based',
    waitTime: 30,
    loopContinuous: true,
    connected: false,
    mapImage: null,
    mapInfo: null,
    processes: {}
};

// ==================== API ====================
const API = {
    baseUrl: '',
    
    async get(endpoint) {
        try {
            const response = await fetch(this.baseUrl + endpoint);
            return await response.json();
        } catch (error) {
            console.error('API GET error:', error);
            return null;
        }
    },
    
    async post(endpoint, data = {}) {
        try {
            const response = await fetch(this.baseUrl + endpoint, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify(data)
            });
            return await response.json();
        } catch (error) {
            console.error('API POST error:', error);
            return null;
        }
    }
};

// ==================== INITIALIZATION ====================
document.addEventListener('DOMContentLoaded', () => {
    initModeCards();
    initMapSelection();
    initWaitModeOptions();
    initJoysticks();
    initKeyboardTeleop();
    startStatusPolling();
    updateTimestamp();
    setInterval(updateTimestamp, 1000);
});

function updateTimestamp() {
    document.getElementById('timestamp').textContent = new Date().toLocaleString();
}

// ==================== MODE SELECTION ====================
function initModeCards() {
    document.querySelectorAll('.mode-card').forEach(card => {
        card.addEventListener('click', () => {
            document.querySelectorAll('.mode-card').forEach(c => c.classList.remove('selected'));
            card.classList.add('selected');
            state.selectedMode = card.dataset.mode;
            
            // Navigate based on mode
            setTimeout(() => {
                if (state.selectedMode === 'waypoint') {
                    loadMaps();
                    showStep('step-map');
                } else if (state.selectedMode === 'detection') {
                    showStep('step-detection-setup');
                } else if (state.selectedMode === 'manual') {
                    showStep('step-manual-setup');
                }
            }, 300);
        });
    });
}

async function setMode(mode) {
    const result = await API.post('/api/mode', { mode });
    if (result && result.success) {
        state.selectedMode = mode;
        document.getElementById('current-mode').textContent = `Mode: ${mode.charAt(0).toUpperCase() + mode.slice(1)}`;
    }
}

// ==================== LAUNCH ORCHESTRATION ====================
async function launchRobot(launchCameras = true) {
    showNotification('Launching robot... Please wait', 'info');
    
    const result = await API.post('/api/launch/robot', {
        launch_cameras: launchCameras,
        hardware_mode: 'real'
    });
    
    if (result && result.success) {
        showNotification('Robot launched successfully!', 'success');
        return true;
    } else {
        showNotification('Failed to launch robot', 'error');
        return false;
    }
}

async function launchMapping() {
    showNotification('Starting SLAM mapping...', 'info');
    
    const result = await API.post('/api/launch/mapping');
    
    if (result && result.success) {
        showNotification('SLAM mapping started!', 'success');
        return true;
    } else {
        showNotification('Failed to start mapping', 'error');
        return false;
    }
}

async function launchLocalization(mapPath) {
    showNotification('Starting localization...', 'info');
    
    const result = await API.post('/api/launch/localization', { map_path: mapPath });
    
    if (result && result.success) {
        showNotification('Localization started!', 'success');
        return true;
    } else {
        showNotification('Failed to start localization', 'error');
        return false;
    }
}

async function launchWaypointNav(mapPath) {
    showNotification('Starting waypoint navigation...', 'info');
    
    const result = await API.post('/api/launch/waypoint_nav', { map_path: mapPath });
    
    if (result && result.success) {
        showNotification('Waypoint navigation ready!', 'success');
        return true;
    } else {
        showNotification('Failed to start waypoint navigation', 'error');
        return false;
    }
}

async function launchDetection(enableAutonomous = false) {
    showNotification('Starting detection mode...', 'info');
    
    const result = await API.post('/api/launch/detection', {
        enable_autonomous_navigation: enableAutonomous
    });
    
    if (result && result.success) {
        showNotification('Detection mode started!', 'success');
        return true;
    } else {
        showNotification('Failed to start detection mode', 'error');
        return false;
    }
}

async function stopLaunch(name) {
    const result = await API.post('/api/launch/stop', { name });
    return result && result.success;
}

async function stopAllLaunches() {
    showNotification('Stopping all processes...', 'info');
    const result = await API.post('/api/launch/stop', { name: 'all' });
    if (result && result.success) {
        showNotification('All processes stopped', 'success');
    }
    return result && result.success;
}

function showNotification(message, type = 'info') {
    // Create notification element
    let notif = document.getElementById('notification');
    if (!notif) {
        notif = document.createElement('div');
        notif.id = 'notification';
        notif.style.cssText = `
            position: fixed;
            top: 20px;
            right: 20px;
            padding: 1rem 1.5rem;
            border-radius: 0.5rem;
            color: white;
            font-weight: 500;
            z-index: 1000;
            transition: opacity 0.3s;
        `;
        document.body.appendChild(notif);
    }
    
    const colors = {
        info: '#2563eb',
        success: '#16a34a',
        warning: '#ea580c',
        error: '#dc2626'
    };
    
    notif.style.background = colors[type] || colors.info;
    notif.textContent = message;
    notif.style.opacity = '1';
    
    setTimeout(() => {
        notif.style.opacity = '0';
    }, 3000);
}

// ==================== WIZARD NAVIGATION ====================
function showStep(stepId) {
    document.querySelectorAll('.wizard-step').forEach(step => {
        step.classList.remove('active');
    });
    
    // Create step if it doesn't exist
    let stepEl = document.getElementById(stepId);
    if (!stepEl) {
        stepEl = createDynamicStep(stepId);
    }
    
    stepEl.classList.add('active');
    state.currentStep = stepId;
}

function createDynamicStep(stepId) {
    const wizard = document.getElementById('wizard');
    const section = document.createElement('section');
    section.id = stepId;
    section.className = 'wizard-step';
    
    if (stepId === 'step-detection-setup') {
        section.innerHTML = `
            <h2>Detection Mode Setup</h2>
            <div class="setup-panel">
                <p>This mode uses AI-powered human detection for autonomous photography.</p>
                
                <div class="setup-steps">
                    <div class="setup-step">
                        <h3>Step 1: Launch Robot Hardware</h3>
                        <p>Launch robot with cameras <strong>disabled</strong> (detection mode uses direct SDK)</p>
                        <button class="btn btn-primary" onclick="launchRobotForDetection()">
                            🤖 Launch Robot (no cameras)
                        </button>
                        <span id="robot-status-detection" class="status-badge">Not started</span>
                    </div>
                    
                    <div class="setup-step">
                        <h3>Step 2: Enable Autonomous Navigation?</h3>
                        <label class="checkbox-option">
                            <input type="checkbox" id="enable-autonomous">
                            <span>Enable autonomous navigation (robot will move to POIs)</span>
                        </label>
                        <p class="warning-text">⚠️ Leave unchecked for testing - robot won't move</p>
                    </div>
                    
                    <div class="setup-step">
                        <h3>Step 3: Start Detection System</h3>
                        <button class="btn btn-success" onclick="startDetectionMode()">
                            👁️ Start Detection Mode
                        </button>
                        <span id="detection-status" class="status-badge">Not started</span>
                    </div>
                </div>
            </div>
            <div class="wizard-nav">
                <button class="btn btn-secondary" onclick="goToModeSelection()">← Back</button>
            </div>
        `;
    } else if (stepId === 'step-manual-setup') {
        section.innerHTML = `
            <h2>Manual Teleop Setup</h2>
            <div class="setup-panel">
                <p>Direct manual control of the robot.</p>
                
                <div class="setup-steps">
                    <div class="setup-step">
                        <h3>Step 1: Launch Robot Hardware</h3>
                        <p>Choose camera option:</p>
                        <div class="btn-group">
                            <button class="btn btn-primary" onclick="launchRobotManual(true)">
                                🤖 With Cameras (3D obstacle avoidance)
                            </button>
                            <button class="btn btn-secondary" onclick="launchRobotManual(false)">
                                🤖 Without Cameras (faster startup)
                            </button>
                        </div>
                        <span id="robot-status-manual" class="status-badge">Not started</span>
                    </div>
                    
                    <div class="setup-step">
                        <h3>Step 2: Optional - Enable Mapping</h3>
                        <label class="checkbox-option">
                            <input type="checkbox" id="enable-mapping-manual">
                            <span>Enable SLAM mapping (build a map while driving)</span>
                        </label>
                        <button class="btn btn-small" onclick="startMappingManual()" id="btn-start-mapping" disabled>
                            Start Mapping
                        </button>
                    </div>
                    
                    <div class="setup-step">
                        <h3>Step 3: Start Teleop</h3>
                        <button class="btn btn-success" onclick="startManualTeleop()">
                            🎮 Open Teleop Controls
                        </button>
                    </div>
                </div>
            </div>
            <div class="wizard-nav">
                <button class="btn btn-secondary" onclick="goToModeSelection()">← Back</button>
            </div>
        `;
    } else if (stepId === 'step-waypoint-setup') {
        section.innerHTML = `
            <h2>Waypoint Navigation Setup</h2>
            <div class="setup-panel">
                <div class="setup-steps">
                    <div class="setup-step">
                        <h3>Step 1: Launch Robot Hardware</h3>
                        <p>Launch robot with cameras <strong>enabled</strong> (for 3D obstacle avoidance)</p>
                        <button class="btn btn-primary" onclick="launchRobotForWaypoint()">
                            🤖 Launch Robot (with cameras)
                        </button>
                        <span id="robot-status-waypoint" class="status-badge">Not started</span>
                    </div>
                    
                    <div class="setup-step">
                        <h3>Step 2: Start Navigation</h3>
                        <p>Selected map: <strong id="selected-map-name">${state.selectedMap || 'None'}</strong></p>
                        <button class="btn btn-success" onclick="startWaypointNavigationSystem()">
                            🗺️ Start Navigation System
                        </button>
                        <span id="nav-status-waypoint" class="status-badge">Not started</span>
                    </div>
                    
                    <div class="setup-step">
                        <h3>Step 3: Select Waypoints</h3>
                        <button class="btn btn-primary" onclick="showStep('step-waypoints')" id="btn-select-waypoints" disabled>
                            📍 Select Waypoints on Map
                        </button>
                    </div>
                </div>
            </div>
            <div class="wizard-nav">
                <button class="btn btn-secondary" onclick="showStep('step-map')">← Back to Map Selection</button>
            </div>
        `;
    }
    
    wizard.appendChild(section);
    return section;
}

// ==================== MODE-SPECIFIC LAUNCH FUNCTIONS ====================

async function launchRobotForDetection() {
    const statusEl = document.getElementById('robot-status-detection');
    statusEl.textContent = 'Launching...';
    statusEl.className = 'status-badge warning';
    
    const success = await launchRobot(false);  // No cameras for detection mode
    
    if (success) {
        statusEl.textContent = 'Running';
        statusEl.className = 'status-badge success';
    } else {
        statusEl.textContent = 'Failed';
        statusEl.className = 'status-badge error';
    }
}

async function startDetectionMode() {
    const enableAutonomous = document.getElementById('enable-autonomous')?.checked || false;
    const statusEl = document.getElementById('detection-status');
    
    statusEl.textContent = 'Starting...';
    statusEl.className = 'status-badge warning';
    
    const success = await launchDetection(enableAutonomous);
    
    if (success) {
        statusEl.textContent = 'Running';
        statusEl.className = 'status-badge success';
        setMode('detection');
        setTimeout(() => showStep('step-detection'), 1000);
    } else {
        statusEl.textContent = 'Failed';
        statusEl.className = 'status-badge error';
    }
}

async function launchRobotManual(withCameras) {
    const statusEl = document.getElementById('robot-status-manual');
    statusEl.textContent = 'Launching...';
    statusEl.className = 'status-badge warning';
    
    const success = await launchRobot(withCameras);
    
    if (success) {
        statusEl.textContent = 'Running';
        statusEl.className = 'status-badge success';
        document.getElementById('btn-start-mapping').disabled = false;
    } else {
        statusEl.textContent = 'Failed';
        statusEl.className = 'status-badge error';
    }
}

async function startMappingManual() {
    if (document.getElementById('enable-mapping-manual')?.checked) {
        await launchMapping();
    }
}

function startManualTeleop() {
    setMode('manual');
    showStep('step-manual');
}

async function launchRobotForWaypoint() {
    const statusEl = document.getElementById('robot-status-waypoint');
    statusEl.textContent = 'Launching...';
    statusEl.className = 'status-badge warning';
    
    const success = await launchRobot(true);  // With cameras for waypoint mode
    
    if (success) {
        statusEl.textContent = 'Running';
        statusEl.className = 'status-badge success';
    } else {
        statusEl.textContent = 'Failed';
        statusEl.className = 'status-badge error';
    }
}

async function startWaypointNavigationSystem() {
    const statusEl = document.getElementById('nav-status-waypoint');
    statusEl.textContent = 'Starting...';
    statusEl.className = 'status-badge warning';
    
    const mapPath = state.selectedMap ? 
        `/home/hype/manriix2_ws/src/manriix_navigation/maps/${state.selectedMap}.yaml` : '';
    
    const success = await launchWaypointNav(mapPath);
    
    if (success) {
        statusEl.textContent = 'Running';
        statusEl.className = 'status-badge success';
        document.getElementById('btn-select-waypoints').disabled = false;
        
        // Wait for map to be published
        setTimeout(() => loadMapImage(), 2000);
    } else {
        statusEl.textContent = 'Failed';
        statusEl.className = 'status-badge error';
    }
}

function wizardNext() {
    const stepOrder = {
        'step-mode': () => {
            if (state.selectedMode === 'waypoint') {
                loadMaps();
                return 'step-map';
            }
            return null;
        },
        'step-map': () => {
            if (state.mapSource === 'new') {
                return 'step-mapping';
            } else if (state.selectedMap) {
                // Go to waypoint setup (launch orchestration)
                return 'step-waypoint-setup';
            }
            alert('Please select a map');
            return null;
        },
        'step-mapping': () => 'step-save-map',
        'step-save-map': () => {
            return 'step-waypoint-setup';
        },
        'step-waypoints': () => {
            if (state.waypoints.length === 0) {
                alert('Please add at least one waypoint');
                return null;
            }
            return 'step-wait-config';
        }
    };
    
    const nextStepFn = stepOrder[state.currentStep];
    if (nextStepFn) {
        const nextStep = nextStepFn();
        if (nextStep) {
            showStep(nextStep);
        }
    }
}

function wizardBack() {
    const backOrder = {
        'step-map': 'step-mode',
        'step-mapping': 'step-map',
        'step-save-map': 'step-mapping',
        'step-waypoint-setup': 'step-map',
        'step-waypoints': 'step-waypoint-setup',
        'step-wait-config': 'step-waypoints',
        'step-navigation': 'step-wait-config',
        'step-manual': 'step-manual-setup',
        'step-manual-setup': 'step-mode',
        'step-detection': 'step-detection-setup',
        'step-detection-setup': 'step-mode'
    };
    
    const prevStep = backOrder[state.currentStep];
    if (prevStep) {
        showStep(prevStep);
    }
}

function goToModeSelection() {
    setMode('idle');
    showStep('step-mode');
}

// ==================== MAP MANAGEMENT ====================
function initMapSelection() {
    document.querySelectorAll('input[name="map-source"]').forEach(radio => {
        radio.addEventListener('change', (e) => {
            state.mapSource = e.target.value;
            document.getElementById('map-list-container').style.display = 
                e.target.value === 'existing' ? 'block' : 'none';
        });
    });
}

async function loadMaps() {
    const container = document.getElementById('map-list-container');
    container.innerHTML = '<p class="loading">Loading maps...</p>';
    
    const result = await API.get('/api/maps');
    
    if (result && result.maps) {
        if (result.maps.length === 0) {
            container.innerHTML = '<p class="loading">No maps available. Please create a new map.</p>';
            document.querySelector('input[name="map-source"][value="new"]').checked = true;
            state.mapSource = 'new';
        } else {
            container.innerHTML = result.maps.map(map => `
                <div class="map-item" data-name="${map.name}" onclick="selectMap('${map.name}')">
                    <div>
                        <div class="map-item-name">${map.name}</div>
                        <div class="map-item-meta">${map.modified} | ${map.size_kb} KB</div>
                    </div>
                </div>
            `).join('');
        }
    } else {
        container.innerHTML = '<p class="loading">Error loading maps</p>';
    }
}

async function selectMap(mapName) {
    state.selectedMap = mapName;
    document.querySelectorAll('.map-item').forEach(item => {
        item.classList.toggle('selected', item.dataset.name === mapName);
    });
    
    // Notify server
    await API.post('/api/maps/select', { name: mapName });
}

async function loadMapImage() {
    const result = await API.get('/api/maps/image');
    if (result && result.success) {
        state.mapImage = result.image;
        state.mapInfo = {
            width: result.width,
            height: result.height,
            resolution: result.resolution,
            origin: result.origin
        };
        drawWaypointCanvas();
    }
}

async function saveMap() {
    const mapName = document.getElementById('map-name').value.trim();
    const statusEl = document.getElementById('save-status');
    
    if (!mapName) {
        statusEl.textContent = 'Please enter a map name';
        statusEl.className = 'status-message error';
        return;
    }
    
    statusEl.textContent = 'Saving map...';
    statusEl.className = 'status-message';
    
    const result = await API.post('/api/maps/save', { name: mapName });
    
    if (result && result.success) {
        statusEl.textContent = `Map "${mapName}" saved successfully!`;
        statusEl.className = 'status-message success';
        state.selectedMap = mapName;
    } else {
        statusEl.textContent = result?.error || 'Error saving map';
        statusEl.className = 'status-message error';
    }
}

async function finishMapping() {
    // Stop mapping launch
    await stopLaunch('mapping');
    showStep('step-save-map');
}

// ==================== WAYPOINT MANAGEMENT ====================
function drawWaypointCanvas() {
    const canvas = document.getElementById('waypoint-canvas');
    if (!canvas) return;
    
    const ctx = canvas.getContext('2d');
    
    ctx.fillStyle = '#333';
    ctx.fillRect(0, 0, canvas.width, canvas.height);
    
    if (state.mapImage) {
        const img = new Image();
        img.onload = () => {
            ctx.drawImage(img, 0, 0, canvas.width, canvas.height);
            drawWaypoints(ctx, canvas);
        };
        img.src = state.mapImage;
    } else {
        ctx.strokeStyle = '#444';
        for (let x = 0; x < canvas.width; x += 50) {
            ctx.beginPath();
            ctx.moveTo(x, 0);
            ctx.lineTo(x, canvas.height);
            ctx.stroke();
        }
        for (let y = 0; y < canvas.height; y += 50) {
            ctx.beginPath();
            ctx.moveTo(0, y);
            ctx.lineTo(canvas.width, y);
            ctx.stroke();
        }
        drawWaypoints(ctx, canvas);
    }
    
    canvas.onclick = (e) => {
        const rect = canvas.getBoundingClientRect();
        const x = e.clientX - rect.left;
        const y = e.clientY - rect.top;
        addWaypointFromCanvas(x, y, canvas);
    };
}

function drawWaypoints(ctx, canvas) {
    state.waypoints.forEach((wp, index) => {
        const canvasX = wp.canvasX || wp.x * 30 + canvas.width / 2;
        const canvasY = wp.canvasY || canvas.height / 2 - wp.y * 30;
        
        ctx.beginPath();
        ctx.arc(canvasX, canvasY, 15, 0, Math.PI * 2);
        ctx.fillStyle = '#2563eb';
        ctx.fill();
        ctx.strokeStyle = 'white';
        ctx.lineWidth = 2;
        ctx.stroke();
        
        ctx.fillStyle = 'white';
        ctx.font = 'bold 12px sans-serif';
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        ctx.fillText(index + 1, canvasX, canvasY);
        
        if (index < state.waypoints.length - 1) {
            const nextWp = state.waypoints[index + 1];
            const nextX = nextWp.canvasX || nextWp.x * 30 + canvas.width / 2;
            const nextY = nextWp.canvasY || canvas.height / 2 - nextWp.y * 30;
            
            ctx.beginPath();
            ctx.moveTo(canvasX, canvasY);
            ctx.lineTo(nextX, nextY);
            ctx.strokeStyle = '#2563eb';
            ctx.lineWidth = 2;
            ctx.setLineDash([5, 5]);
            ctx.stroke();
            ctx.setLineDash([]);
        }
    });
}

function addWaypointFromCanvas(canvasX, canvasY, canvas) {
    let worldX, worldY;
    
    if (state.mapInfo) {
        const scaleX = state.mapInfo.width / canvas.width;
        const scaleY = state.mapInfo.height / canvas.height;
        worldX = (canvasX * scaleX) * state.mapInfo.resolution + state.mapInfo.origin.x;
        worldY = ((canvas.height - canvasY) * scaleY) * state.mapInfo.resolution + state.mapInfo.origin.y;
    } else {
        worldX = (canvasX - canvas.width / 2) / 30;
        worldY = (canvas.height / 2 - canvasY) / 30;
    }
    
    const waypoint = {
        x: parseFloat(worldX.toFixed(2)),
        y: parseFloat(worldY.toFixed(2)),
        theta: 0,
        name: `WP${state.waypoints.length + 1}`,
        canvasX: canvasX,
        canvasY: canvasY
    };
    
    state.waypoints.push(waypoint);
    updateWaypointList();
    drawWaypointCanvas();
}

function updateWaypointList() {
    const listEl = document.getElementById('waypoint-list-ul');
    const countEl = document.getElementById('waypoint-count');
    
    if (countEl) countEl.textContent = state.waypoints.length;
    
    if (listEl) {
        listEl.innerHTML = state.waypoints.map((wp, index) => `
            <li>
                <span class="wp-number">${index + 1}</span>
                <span class="wp-coords">(${wp.x.toFixed(1)}, ${wp.y.toFixed(1)})</span>
                <span class="wp-remove" onclick="removeWaypoint(${index})">✕</span>
            </li>
        `).join('');
    }
}

function removeWaypoint(index) {
    state.waypoints.splice(index, 1);
    state.waypoints.forEach((wp, i) => {
        wp.name = `WP${i + 1}`;
    });
    updateWaypointList();
    drawWaypointCanvas();
}

function clearWaypoints() {
    state.waypoints = [];
    updateWaypointList();
    drawWaypointCanvas();
}

function undoWaypoint() {
    if (state.waypoints.length > 0) {
        state.waypoints.pop();
        updateWaypointList();
        drawWaypointCanvas();
    }
}

// ==================== WAIT MODE ====================
function initWaitModeOptions() {
    document.querySelectorAll('input[name="wait-mode"]').forEach(radio => {
        radio.addEventListener('change', (e) => {
            state.waitMode = e.target.value;
            const timeOptions = document.getElementById('time-options');
            if (timeOptions) {
                timeOptions.style.display = e.target.value === 'time_based' ? 'block' : 'none';
            }
        });
    });
    
    document.querySelectorAll('.time-btn').forEach(btn => {
        btn.addEventListener('click', () => {
            document.querySelectorAll('.time-btn').forEach(b => b.classList.remove('active'));
            btn.classList.add('active');
            state.waitTime = parseInt(btn.dataset.time);
            const customInput = document.getElementById('custom-wait-time');
            if (customInput) customInput.value = state.waitTime;
        });
    });
    
    const customTimeInput = document.getElementById('custom-wait-time');
    if (customTimeInput) {
        customTimeInput.addEventListener('change', (e) => {
            state.waitTime = parseInt(e.target.value) || 30;
            document.querySelectorAll('.time-btn').forEach(b => b.classList.remove('active'));
        });
    }
    
    const loopCheckbox = document.getElementById('loop-continuous');
    if (loopCheckbox) {
        loopCheckbox.addEventListener('change', (e) => {
            state.loopContinuous = e.target.checked;
        });
    }
}

// ==================== NAVIGATION CONTROL ====================
async function startWaypointNavigation() {
    await API.post('/api/waypoints', { waypoints: state.waypoints });
    
    await API.post('/api/waypoints/config', {
        wait_mode: state.waitMode,
        wait_time: state.waitTime,
        loop_continuous: state.loopContinuous
    });
    
    const result = await API.post('/api/waypoints/control', { command: 'start' });
    
    if (result && result.success) {
        setMode('waypoint');
        showStep('step-navigation');
    } else {
        alert('Failed to start navigation');
    }
}

async function pauseNavigation() {
    await API.post('/api/waypoints/control', { command: 'pause' });
}

async function skipWaypoint() {
    await API.post('/api/waypoints/control', { command: 'skip' });
}

async function stopNavigation() {
    await API.post('/api/waypoints/control', { command: 'stop' });
    showStep('step-wait-config');
}

// ==================== TELEOP ====================
function initJoysticks() {
    setupJoystick('manual-joystick');
}

function setupJoystick(containerId) {
    const container = document.getElementById(containerId);
    if (!container) return;
    
    const handle = container.querySelector('.joystick-handle');
    const base = container.querySelector('.joystick-base');
    
    let isDragging = false;
    let centerX, centerY, maxRadius;
    
    function updateJoystickPosition(clientX, clientY) {
        const rect = base.getBoundingClientRect();
        centerX = rect.left + rect.width / 2;
        centerY = rect.top + rect.height / 2;
        maxRadius = rect.width / 4;
        
        let dx = clientX - centerX;
        let dy = clientY - centerY;
        
        const distance = Math.sqrt(dx * dx + dy * dy);
        if (distance > maxRadius) {
            dx = dx / distance * maxRadius;
            dy = dy / distance * maxRadius;
        }
        
        handle.style.transform = `translate(${dx}px, ${dy}px)`;
        
        const linearX = -dy / maxRadius * 0.5;
        const angularZ = -dx / maxRadius * 1.0;
        
        const linearEl = document.getElementById('linear-vel');
        const angularEl = document.getElementById('angular-vel');
        if (linearEl) linearEl.textContent = linearX.toFixed(2);
        if (angularEl) angularEl.textContent = angularZ.toFixed(2);
        
        API.post('/api/teleop', { linear_x: linearX, angular_z: angularZ });
    }
    
    function resetJoystick() {
        handle.style.transform = 'translate(0, 0)';
        const linearEl = document.getElementById('linear-vel');
        const angularEl = document.getElementById('angular-vel');
        if (linearEl) linearEl.textContent = '0.00';
        if (angularEl) angularEl.textContent = '0.00';
        API.post('/api/teleop/stop');
    }
    
    handle.addEventListener('mousedown', (e) => {
        isDragging = true;
        e.preventDefault();
    });
    
    document.addEventListener('mousemove', (e) => {
        if (isDragging) {
            updateJoystickPosition(e.clientX, e.clientY);
        }
    });
    
    document.addEventListener('mouseup', () => {
        if (isDragging) {
            isDragging = false;
            resetJoystick();
        }
    });
    
    handle.addEventListener('touchstart', (e) => {
        isDragging = true;
        e.preventDefault();
    });
    
    document.addEventListener('touchmove', (e) => {
        if (isDragging && e.touches.length > 0) {
            updateJoystickPosition(e.touches[0].clientX, e.touches[0].clientY);
        }
    });
    
    document.addEventListener('touchend', () => {
        if (isDragging) {
            isDragging = false;
            resetJoystick();
        }
    });
}

function initKeyboardTeleop() {
    const keys = {};
    
    document.addEventListener('keydown', (e) => {
        if (state.currentStep !== 'step-manual' && state.currentStep !== 'step-mapping') return;
        
        keys[e.key.toLowerCase()] = true;
        updateTeleopFromKeys(keys);
    });
    
    document.addEventListener('keyup', (e) => {
        keys[e.key.toLowerCase()] = false;
        updateTeleopFromKeys(keys);
    });
}

function updateTeleopFromKeys(keys) {
    let linearX = 0;
    let angularZ = 0;
    
    if (keys['w']) linearX += 0.3;
    if (keys['s']) linearX -= 0.3;
    if (keys['a']) angularZ += 0.5;
    if (keys['d']) angularZ -= 0.5;
    
    if (linearX !== 0 || angularZ !== 0) {
        API.post('/api/teleop', { linear_x: linearX, angular_z: angularZ });
    } else {
        API.post('/api/teleop/stop');
    }
    
    const linearEl = document.getElementById('linear-vel');
    const angularEl = document.getElementById('angular-vel');
    if (linearEl) linearEl.textContent = linearX.toFixed(2);
    if (angularEl) angularEl.textContent = angularZ.toFixed(2);
}

async function emergencyStop() {
    await API.post('/api/teleop/stop');
    await API.post('/api/waypoints/control', { command: 'stop' });
    showNotification('Emergency stop activated!', 'warning');
}

// ==================== STATUS POLLING ====================
function startStatusPolling() {
    setInterval(async () => {
        const result = await API.get('/api/status');
        if (result) {
            updateStatusDisplay(result);
            state.connected = true;
            state.processes = result.status?.processes || {};
            document.getElementById('connection-status').className = 'status-indicator connected';
            document.getElementById('connection-status').textContent = '● Connected';
        } else {
            state.connected = false;
            document.getElementById('connection-status').className = 'status-indicator disconnected';
            document.getElementById('connection-status').textContent = '● Disconnected';
        }
    }, 1000);
}

function updateStatusDisplay(data) {
    if (data.mode) {
        const modeText = data.mode.charAt(0).toUpperCase() + data.mode.slice(1);
        document.getElementById('current-mode').textContent = `Mode: ${modeText}`;
    }
    
    if (state.currentStep === 'step-navigation' && data.status) {
        const wpStatus = data.status.waypoint_status;
        if (wpStatus) {
            const navStatusEl = document.getElementById('nav-status');
            if (navStatusEl) navStatusEl.textContent = wpStatus;
            
            const match = wpStatus.match(/(\d+)\/(\d+)/);
            if (match) {
                const current = parseInt(match[1]);
                const total = parseInt(match[2]);
                const currentWpEl = document.getElementById('nav-current-wp');
                const progressEl = document.getElementById('nav-progress');
                if (currentWpEl) currentWpEl.textContent = `${current} / ${total}`;
                if (progressEl) progressEl.style.width = `${(current / total) * 100}%`;
            }
        }
    }
    
    if (state.currentStep === 'step-detection' && data.status) {
        if (data.status.mission_status) {
            const missionEl = document.getElementById('mission-state');
            if (missionEl) missionEl.textContent = data.status.mission_status;
        }
    }
}
