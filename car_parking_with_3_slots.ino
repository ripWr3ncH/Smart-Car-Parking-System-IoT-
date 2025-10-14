#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <ESP32Servo.h>

// ---------------- WiFi Config ----------------
const char* WIFI_SSID = "AEH_E6_407";
const char* WIFI_PASSWORD = "30393257";
const char* MDNS_NAME = "esp32-parking"; // http://esp32-parking.local

// ---------------- Pin Setup ----------------
#define IR_ENTRY 19
#define IR_EXIT 18
#define IR_SLOT1 16             // Changed from 21 to G16
#define IR_SLOT2 17
#define IR_SLOT3 4              // New IR sensor on G4
#define SERVO_ENTRY_PIN 25      // Entry gate servo
#define SERVO_EXIT_PIN 26       // Exit gate servo

// ---------------- Hardware ----------------
Servo entryServo;               // Entry gate servo motor
Servo exitServo;                // Exit gate servo motor

// ---------------- Parking Variables ----------------
const int totalSlots = 3;       // Now 3 slots
bool slotOccupied[totalSlots] = {false, false, false};
bool lastSlotOccupied[totalSlots] = {false, false, false};
bool slotBlocked[totalSlots] = {false, false, false}; // Blocks slot until payment cleared
int availableSlots = totalSlots;

// ---------------- Gate/Servo ----------------
// Entry gate servo angles
int entryServoClosed = 90;      // Adjust for your linkage
int entryServoOpen   = 0;       // Adjust for your linkage

// Exit gate servo angles
int exitServoClosed = 90;       // Adjust for your linkage
int exitServoOpen   = 0;        // Adjust for your linkage

// Entry servo state
volatile int entryCurrentAngle = entryServoClosed;
volatile int entryTargetAngle  = entryServoClosed;

// Exit servo state
volatile int exitCurrentAngle = exitServoClosed;
volatile int exitTargetAngle  = exitServoClosed;

const int servoStep = 2;        // deg per step
const int servoStepDelayMs = 25;
unsigned long lastServoStepMs = 0;

String entryGateStatus = "Closed";   // Closed, Opening, Open, Closing
String exitGateStatus = "Closed";    // Closed, Opening, Open, Closing

// ---------------- Logic Flags ----------------
bool manualOverride = false;    // When true, auto logic won't move gate
bool entryActive = false;       // IR entry beam broken
bool exitActive  = false;       // IR exit beam broken
bool camCarDetected = false;    // Car detected by ESP32-CAM
unsigned long lastCamDetection = 0;
const unsigned long camDetectionTimeout = 8000; // 8 seconds timeout
bool gateOpenedForEntry = false; // Track if gate was opened for an entry
bool exitPaymentProcessed = false; // Track if payment was already processed for current exit
bool lastExitActive = false;    // Track previous exit IR state

// ---------------- Web Server ----------------
WebServer server(80);

// ---------------- HTML UI (inline) ----------------
const char INDEX_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ESP32 Parking IoT</title>
<style>
  :root{--bg:#0e1116;--card:#171b22;--fg:#e6edf3;--muted:#9aa4b2;--ok:#2ea043;--bad:#f85149;--warn:#d29922;--accent:#4493f8}
  body{margin:0;background:var(--bg);color:var(--fg);font-family:system-ui,-apple-system,Segoe UI,Roboto,Arial}
  header{padding:16px 20px;background:#11161e;border-bottom:1px solid #232a33}
  h1{font-size:18px;margin:0}
  main{padding:20px;display:grid;gap:16px;grid-template-columns:repeat(auto-fit,minmax(280px,1fr))}
  .card{background:var(--card);border:1px solid #232a33;border-radius:10px;padding:16px}
  .row{display:flex;gap:12px;align-items:center;flex-wrap:wrap}
  .tag{padding:4px 10px;border-radius:999px;font-size:12px;border:1px solid #2b323c;color:var(--muted)}
  .tag.ok{border-color:#294b33;color:var(--ok)}
  .tag.bad{border-color:#5a2b2b;color:var(--bad)}
  .tag.warn{border-color:#5a4b2b;color:var(--warn)}
  .big{font-size:28px;font-weight:700}
  .muted{color:var(--muted)}
  .slots{display:flex;gap:10px;margin-top:10px}
  .slot{flex:1;min-width:100px;border:1px dashed #2b323c;border-radius:8px;padding:10px;text-align:center}
  .slot.free{border-color:#294b33;color:var(--ok)}
  .slot.occ{border-color:#5a2b2b;color:var(--bad)}
  button{background:var(--accent);border:none;color:white;padding:10px 14px;border-radius:8px;cursor:pointer}
  button.alt{background:#2b323c}
  button:disabled{opacity:.6;cursor:not-allowed}
  .grid{display:grid;grid-template-columns:1fr 1fr;gap:10px}
  code{background:#0b0e13;border:1px solid #232a33;border-radius:6px;padding:2px 6px}
  table{width:100%;border-collapse:collapse;margin-top:10px}
  th,td{border:1px solid #232a33;padding:8px;text-align:center}
  th{background:#11161e;color:var(--muted);font-weight:600}
  .cost{color:var(--warn);font-weight:600}
  .present{color:var(--ok)}
  .absent{color:var(--muted)}
  .payment-notification{
    position:fixed;
    top:50%;
    left:50%;
    transform:translate(-50%,-50%);
    background:var(--warn);
    color:#000;
    padding:30px 40px;
    border-radius:15px;
    font-size:36px;
    font-weight:700;
    text-align:center;
    box-shadow:0 10px 30px rgba(0,0,0,0.5);
    z-index:1000;
    display:none;
    animation:pulse 2s infinite;
    max-width:90vw;
    max-height:90vh;
    overflow-y:auto;
  }
  .payment-notification.show{display:block}
  .payment-item{
    margin-bottom:20px;
    padding:20px;
    background:rgba(0,0,0,0.1);
    border-radius:10px;
    border:2px solid transparent;
  }
  .payment-item.priority{
    border-color:#ff6b35;
    background:rgba(255,107,53,0.2);
  }
  .payment-amount{
    font-size:42px;
    font-weight:800;
    margin-bottom:10px;
  }
  .payment-details{
    font-size:18px;
    font-weight:400;
    margin-bottom:5px;
  }
  .payment-priority{
    font-size:14px;
    font-weight:600;
    color:#ff6b35;
    margin-top:10px;
  }
  .payment-queue-header{
    font-size:24px;
    margin-bottom:20px;
    color:#000;
  }
  @keyframes pulse{
    0%{transform:translate(-50%,-50%) scale(1)}
    50%{transform:translate(-50%,-50%) scale(1.05)}
    100%{transform:translate(-50%,-50%) scale(1)}
  }
  .overlay{
    position:fixed;
    top:0;
    left:0;
    width:100%;
    height:100%;
    background:rgba(0,0,0,0.7);
    z-index:999;
    display:none;
  }
  .overlay.show{display:block}
  @media (max-width: 768px) {
    main{grid-template-columns:1fr;padding:10px}
    .grid{grid-template-columns:1fr}
    table{font-size:14px}
    th,td{padding:6px 4px}
  }
</style>
</head>
<body>
<header><h1>🚗 ESP32 Parking IoT</h1></header>

<!-- Payment Notification Overlay -->
<div class="overlay" id="paymentOverlay"></div>
<div class="payment-notification" id="paymentNotification">
  <div class="payment-queue-header" id="paymentQueueHeader">Payment Queue</div>
  <div id="paymentQueue">
    <!-- Payment items will be populated by JavaScript -->
  </div>
  <div style="font-size:16px;margin-top:20px;font-weight:400;color:#000;">
    Drive to exit gate to complete payment (First out, first served)
  </div>
</div>

<main>
  <section class="card" id="summary">
    <div class="row">
      <div class="big" id="avail">--/--</div>
      <span class="tag" id="gate">Gate: ...</span>
      <span class="tag" id="manual">Auto</span>
    </div>
    <div class="slots" id="slots"></div>
  </section>

  <section class="card">
    <h3>💰 Car Information & Costs</h3>
    <table id="carTable">
      <thead>
        <tr>
          <th>Slot</th>
          <th>Car ID</th>
          <th>Status</th>
          <th>Duration</th>
          <th>Cost (tk)</th>
        </tr>
      </thead>
      <tbody>
        <tr><td colspan="5" class="muted">Loading car data...</td></tr>
      </tbody>
    </table>
  </section>

  <section class="card">
    <h3>🎛 Controls</h3>
    <div style="padding:10px;background:rgba(255,169,0,0.1);border-radius:8px;margin-bottom:10px;font-size:13px;color:var(--warn);" id="modeWarning">
      ⚠️ Manual mode is for testing only. Gate will NOT respond to sensors.
    </div>
    <div class="row" style="margin:10px 0">
      <button id="btn-auto" class="alt" onclick="setManual(0)">🤖 Auto Mode</button>
      <button id="btn-manual" class="alt" onclick="setManual(1)">🛠️ Manual Mode</button>
    </div>
    <div class="row">
      <button id="open" onclick="post('/api/open')" disabled>▲ Open Gate</button>
      <button id="close" class="alt" onclick="post('/api/close')" disabled>▼ Close Gate</button>
    </div>
  </section>

  <section class="card">
    <h3>📊 Diagnostics</h3>
    <div class="grid">
      <div>IR Entry: <code id="ir_entry">-</code></div>
      <div>IR Exit: <code id="ir_exit">-</code></div>
      <div>IR Slot 1: <code id="ir_s1">-</code></div>
      <div>IR Slot 2: <code id="ir_s2">-</code></div>
      <div>IR Slot 3: <code id="ir_s3">-</code></div>
      <div>Entry Servo: <code id="entry_ang">-</code></div>
      <div>Exit Servo: <code id="exit_ang">-</code></div>
      <div>Entry Target: <code id="entry_tang">-</code></div>
      <div>Exit Target: <code id="exit_tang">-</code></div>
      <div>Camera Car: <code id="cam_car">-</code></div>
      <div>Last Cam Detection: <code id="cam_time">-</code></div>
      <div>IP: <code id="ip">-</code></div>
      <div>mDNS: <code id="mdns">-</code></div>
    </div>
  </section>
</main>
<script>
let total = 0;
function el(id){return document.getElementById(id)}
function cls(elm, on, c){ if(on) elm.classList.add(c); else elm.classList.remove(c); }

async function fetchStatus(){
  try{
    const r = await fetch('/status');
    const j = await r.json();
    total = j.total;
    el('avail').textContent = `${j.available}/${j.total} free`;
    const gate = el('gate');
    gate.textContent = `Entry: ${j.entryGate} | Exit: ${j.exitGate}`;
    const entryOpen = j.entryGate === 'Open';
    const exitOpen = j.exitGate === 'Open';
    gate.className = 'tag ' + ((entryOpen || exitOpen) ? 'ok' : '');

    const manual = el('manual');
    manual.textContent = j.manual ? 'Manual' : 'Auto';
    manual.className = 'tag ' + (j.manual?'warn':'');

    // Update mode buttons to show active state
    const btnAuto = el('btn-auto');
    const btnManual = el('btn-manual');
    if (j.manual) {
      btnAuto.style.opacity = '0.5';
      btnManual.style.opacity = '1';
      btnManual.style.background = 'var(--warn)';
    } else {
      btnAuto.style.opacity = '1';
      btnAuto.style.background = 'var(--ok)';
      btnManual.style.opacity = '0.5';
      btnManual.style.background = '#2b323c';
    }

    // Show/hide mode warning
    const modeWarning = el('modeWarning');
    if (j.manual) {
      modeWarning.style.display = 'block';
      modeWarning.style.borderLeft = '3px solid var(--warn)';
    } else {
      modeWarning.style.display = 'none';
    }

    // enable manual buttons only in manual mode
    el('open').disabled = !j.manual;
    el('close').disabled = !j.manual;

    // slots
    const box = el('slots'); box.innerHTML = '';
    j.slots.forEach((occ, idx)=>{
      const d = document.createElement('div');
      d.className = 'slot ' + (occ?'occ':'free');
      d.textContent = `Slot ${idx+1}: ` + (occ?'Occupied':'Free');
      box.appendChild(d);
    });

    // car information table
    const tbody = el('carTable').getElementsByTagName('tbody')[0];
    tbody.innerHTML = '';
    if (j.cars && j.cars.length > 0) {
      j.cars.forEach(car => {
        const row = document.createElement('tr');
        const statusClass = car.present ? 'present' : 'absent';
        const statusText = car.present ? '🚗 Present' : (car.awaitingPayment ? '💳 Payment Due' : '⬜ Empty');
        const costDisplay = car.cost > 0 ? `${car.cost} tk` : '-';
        const durationDisplay = car.duration > 0 ? `${car.duration}s` : '-';
        const carIdDisplay = car.carId > 0 ? `#${car.carId}` : '-';
        
        row.innerHTML = `
          <td>Slot ${car.slot}</td>
          <td>${carIdDisplay}</td>
          <td class="${statusClass}">${statusText}</td>
          <td>${durationDisplay}</td>
          <td class="cost">${costDisplay}</td>
        `;
        tbody.appendChild(row);
      });
    } else {
      tbody.innerHTML = '<tr><td colspan="5" class="muted">No car data available</td></tr>';
    }

    // Handle payment queue notifications
    const overlay = el('paymentOverlay');
    const notification = el('paymentNotification');
    const queueContainer = el('paymentQueue');
    const queueHeader = el('paymentQueueHeader');
    
    if (j.paymentQueue && j.paymentQueue.length > 0) {
      // Update header
      queueHeader.textContent = j.paymentQueue.length === 1 ? 
        'Payment Due' : `Payment Queue (${j.paymentQueue.length} cars)`;
      
      // Clear existing items
      queueContainer.innerHTML = '';
      
      // Add each payment item
      j.paymentQueue.forEach((payment, index) => {
        const item = document.createElement('div');
        item.className = 'payment-item' + (index === 0 ? ' priority' : '');
        
        const priorityText = index === 0 ? 'NEXT TO PAY' : `Position ${index + 1}`;
        const priorityColor = index === 0 ? '#ff6b35' : '#666';
        
        item.innerHTML = `
          <div class="payment-amount">${payment.cost}tk</div>
          <div class="payment-details">Car #${payment.carId} from Slot ${payment.slot}</div>
          <div class="payment-priority" style="color:${priorityColor}">${priorityText}</div>
        `;
        
        queueContainer.appendChild(item);
      });
      
      overlay.classList.add('show');
      notification.classList.add('show');
    } else {
      overlay.classList.remove('show');
      notification.classList.remove('show');
    }

    el('ir_entry').textContent = j.ir.entry===0?'LOW':'HIGH';
    el('ir_exit').textContent  = j.ir.exit===0?'LOW':'HIGH';
    el('ir_s1').textContent    = j.ir.slot1===0?'LOW':'HIGH';
    el('ir_s2').textContent    = j.ir.slot2===0?'LOW':'HIGH';
    el('ir_s3').textContent    = j.ir.slot3===0?'LOW':'HIGH';
    el('entry_ang').textContent = j.entryAngle;
    el('exit_ang').textContent = j.exitAngle;
    el('entry_tang').textContent = j.entryTarget;
    el('exit_tang').textContent = j.exitTarget;
    
    // Camera detection status
    if (j.camera) {
      el('cam_car').textContent = j.camera.carDetected ? 'DETECTED' : 'NONE';
      el('cam_car').style.color = j.camera.carDetected ? 'var(--ok)' : 'var(--muted)';
      if (j.camera.lastDetection > 0) {
        const timeSince = Math.floor((Date.now() - j.camera.lastDetection) / 1000);
        el('cam_time').textContent = `${timeSince}s ago`;
      } else {
        el('cam_time').textContent = 'Never';
      }
    }
    
    el('ip').textContent = j.ip || '-';
    el('mdns').textContent = j.mdns || '-';
  }catch(e){
    console.error('Fetch error:', e);
  }
}

function post(path){
  fetch(path, {method:'POST'})
    .then(response => {
      if (!response.ok) {
        return response.text().then(text => {
          alert('❌ ' + text);
          throw new Error(text);
        });
      }
      return response;
    })
    .then(fetchStatus)
    .catch(err => console.error('Error:', err));
}
function setManual(on){
  fetch('/api/manual?enable='+(on?1:0), {method:'POST'})
    .then(response => {
      if (on) {
        console.log('⚠️ Manual mode enabled - Auto detection disabled');
      } else {
        console.log('✅ Auto mode enabled - Sensors active');
      }
      return response;
    })
    .then(fetchStatus);
}
setInterval(fetchStatus, 1000);
fetchStatus();
</script>
</body>
</html>
)HTML";

// ---------------- Helpers ----------------
void requestEntryServo(int angle) {
  angle = constrain(angle, 0, 180);
  entryTargetAngle = angle;
  if (entryTargetAngle > entryCurrentAngle) entryGateStatus = "Opening";
  else if (entryTargetAngle < entryCurrentAngle) entryGateStatus = "Closing";
  else entryGateStatus = (entryTargetAngle == entryServoOpen) ? "Open" : "Closed";
}

void requestExitServo(int angle) {
  angle = constrain(angle, 0, 180);
  exitTargetAngle = angle;
  if (exitTargetAngle > exitCurrentAngle) exitGateStatus = "Opening";
  else if (exitTargetAngle < exitCurrentAngle) exitGateStatus = "Closing";
  else exitGateStatus = (exitTargetAngle == exitServoOpen) ? "Open" : "Closed";
}

void updateServos() {
  unsigned long now = millis();
  
  // Update entry servo
  if (entryCurrentAngle != entryTargetAngle) {
    if (now - lastServoStepMs >= (unsigned long)servoStepDelayMs) {
      if (entryCurrentAngle < entryTargetAngle)
        entryCurrentAngle = ((entryCurrentAngle + servoStep) > entryTargetAngle) ? entryTargetAngle : (entryCurrentAngle + servoStep);
      else
        entryCurrentAngle = ((entryCurrentAngle - servoStep) < entryTargetAngle) ? entryTargetAngle : (entryCurrentAngle - servoStep);
      entryServo.write(entryCurrentAngle);
      lastServoStepMs = now;
    }
  } else {
    entryGateStatus = (entryCurrentAngle == entryServoOpen) ? "Open" : "Closed";
  }
  
  // Update exit servo
  if (exitCurrentAngle != exitTargetAngle) {
    if (now - lastServoStepMs >= (unsigned long)servoStepDelayMs) {
      if (exitCurrentAngle < exitTargetAngle)
        exitCurrentAngle = ((exitCurrentAngle + servoStep) > exitTargetAngle) ? exitTargetAngle : (exitCurrentAngle + servoStep);
      else
        exitCurrentAngle = ((exitCurrentAngle - servoStep) < exitTargetAngle) ? exitTargetAngle : (exitCurrentAngle - servoStep);
      exitServo.write(exitCurrentAngle);
      lastServoStepMs = now;
    }
  } else {
    exitGateStatus = (exitCurrentAngle == exitServoOpen) ? "Open" : "Closed";
  }
}

struct Sensors {
  int entryRaw;
  int exitRaw;
  int slot1Raw;
  int slot2Raw;
  int slot3Raw;
} sensors;

struct CarInfo {
  unsigned long entryTime = 0; // millis when car entered
  unsigned long exitTime = 0;  // millis when car exited
  unsigned long cost = 0;      // cost in tk
  unsigned long duration = 0;  // seconds
  unsigned long carId = 0;     // unique car id
  bool present = false;
  bool awaitingPayment = false; // true when car left slot but hasn't paid at exit
};
CarInfo cars[3];                // Now 3 slots
unsigned long carIdCounter = 1;

// Payment notification variables
struct PaymentNotification {
  bool active = false;
  unsigned long carId = 0;
  unsigned long cost = 0;
  int slot = 0;
  unsigned long exitTime = 0; // To track order of exit
};

// Payment queue system
const int MAX_PAYMENT_QUEUE = 4; // Support up to 4 pending payments
PaymentNotification paymentQueue[MAX_PAYMENT_QUEUE];
int paymentQueueSize = 0;

// Helper functions for payment queue
void addToPaymentQueue(unsigned long carId, unsigned long cost, int slot, unsigned long exitTime) {
  if (paymentQueueSize < MAX_PAYMENT_QUEUE) {
    paymentQueue[paymentQueueSize].active = true;
    paymentQueue[paymentQueueSize].carId = carId;
    paymentQueue[paymentQueueSize].cost = cost;
    paymentQueue[paymentQueueSize].slot = slot;
    paymentQueue[paymentQueueSize].exitTime = exitTime;
    paymentQueueSize++;
  }
}

void removeFromPaymentQueue(unsigned long carId) {
  for (int i = 0; i < paymentQueueSize; i++) {
    if (paymentQueue[i].carId == carId) {
      // Shift remaining items down
      for (int j = i; j < paymentQueueSize - 1; j++) {
        paymentQueue[j] = paymentQueue[j + 1];
      }
      paymentQueueSize--;
      // Clear the last slot
      if (paymentQueueSize < MAX_PAYMENT_QUEUE) {
        paymentQueue[paymentQueueSize].active = false;
        paymentQueue[paymentQueueSize].carId = 0;
        paymentQueue[paymentQueueSize].cost = 0;
        paymentQueue[paymentQueueSize].slot = 0;
        paymentQueue[paymentQueueSize].exitTime = 0;
      }
      break;
    }
  }
}

unsigned long lastSenseMs = 0;
const unsigned long senseIntervalMs = 100;

// Debouncing variables for slot sensors
bool lastSlotRaw[3] = {HIGH, HIGH, HIGH};
unsigned long slotDebounceStartTime[3] = {0, 0, 0};
const unsigned long debounceDelayMs = 300; // 300ms stable reading required

void readSensorsAndUpdateSlots() {
  unsigned long now = millis();
  if (now - lastSenseMs < senseIntervalMs) return;
  lastSenseMs = now;

  sensors.entryRaw = digitalRead(IR_ENTRY);
  sensors.exitRaw  = digitalRead(IR_EXIT);
  sensors.slot1Raw = digitalRead(IR_SLOT1);
  sensors.slot2Raw = digitalRead(IR_SLOT2);
  sensors.slot3Raw = digitalRead(IR_SLOT3);

  // Store current raw readings in an array for easier processing
  bool currentSlotRaw[3] = {sensors.slot1Raw, sensors.slot2Raw, sensors.slot3Raw};
  
  bool prevOccupied[3] = {slotOccupied[0], slotOccupied[1], slotOccupied[2]};
  
  // Debounced slot reading - require stable state for debounceDelayMs
  for (int i = 0; i < 3; i++) {
    if (currentSlotRaw[i] != lastSlotRaw[i]) {
      // State changed, start debounce timer
      slotDebounceStartTime[i] = now;
      lastSlotRaw[i] = currentSlotRaw[i];
    } else {
      // State is same as last reading
      if (now - slotDebounceStartTime[i] >= debounceDelayMs) {
        // State has been stable for debounceDelayMs, update slot occupancy
        // Active LOW for these common IR modules
        slotOccupied[i] = (currentSlotRaw[i] == LOW);
      }
    }
  }

  availableSlots = totalSlots;
  for (int i = 0; i < totalSlots; i++) {
    if (slotOccupied[i] || slotBlocked[i]) availableSlots--; // Count both occupied AND blocked slots
  }

  entryActive = (sensors.entryRaw == LOW);
  exitActive  = (sensors.exitRaw  == LOW);

  // Check camera detection timeout
  if (camCarDetected && (now - lastCamDetection > camDetectionTimeout)) {
    camCarDetected = false;
    Serial.println("📸 Camera car detection timeout");
  }

  // Car entry/exit and cost logic
  for (int i = 0; i < totalSlots; i++) {
    if (!prevOccupied[i] && slotOccupied[i]) {
      // Car just entered slot - ONLY if slot is NOT blocked by pending payment
      if (!slotBlocked[i]) {
        Serial.printf("✅ Car entering Slot %d (slot is available)\n", i + 1);
        cars[i].entryTime = millis();
        cars[i].carId = carIdCounter++;
        cars[i].present = true;
        cars[i].exitTime = 0;
        cars[i].cost = 0;
        cars[i].duration = 0;
        cars[i].awaitingPayment = false;
      } else {
        // Slot is blocked due to unpaid payment - IGNORE this entry
        Serial.printf("🚫 Slot %d is BLOCKED (payment pending) - Car entry IGNORED\n", i + 1);
        Serial.println("⚠️ Previous car must complete payment before slot can be reused");
      }
    }
    if (prevOccupied[i] && !slotOccupied[i] && cars[i].present) {
      // Car just left slot
      Serial.printf("🚗 Car #%lu leaving Slot %d\n", cars[i].carId, i + 1);
      cars[i].exitTime = millis();
      cars[i].duration = (cars[i].exitTime - cars[i].entryTime) / 1000;
      cars[i].cost = cars[i].duration; // 1tk per second
      cars[i].present = false;
      cars[i].awaitingPayment = true;
      
      // BLOCK this slot until payment is completed
      slotBlocked[i] = true;
      Serial.printf("🔒 Slot %d is now BLOCKED until payment cleared\n", i + 1);
      
      // Add to payment queue (prioritized by exit time - first out, first served)
      addToPaymentQueue(cars[i].carId, cars[i].cost, i + 1, cars[i].exitTime);
    }
    // If car is present, update duration/cost live
    if (slotOccupied[i] && cars[i].present) {
      cars[i].duration = (millis() - cars[i].entryTime) / 1000;
      cars[i].cost = cars[i].duration;
    }
  }
}

void autoGateLogic() {
  if (manualOverride) return;

  // Detect exit IR state change (rising edge - LOW to HIGH transition means car just arrived)
  if (exitActive && !lastExitActive) {
    // Exit IR just triggered (car just arrived at exit)
    exitPaymentProcessed = false; // Reset flag for new exit event
    Serial.println("\n🚗 Car arrived at exit gate - ready to process payment");
  }
  
  // Process payment ONLY ONCE per exit event
  if (exitActive && !exitPaymentProcessed && paymentQueueSize > 0) {
    // Process the first payment in queue (FIFO - first car that left gets priority)
    PaymentNotification firstPayment = paymentQueue[0];
    
    Serial.println("\n💳 ========== Processing Payment ==========");
    Serial.printf("Car #%lu from Slot %d - Cost: %lu tk\n", 
                  firstPayment.carId, firstPayment.slot, firstPayment.cost);
    
    // Find and clear ONLY the specific car that matches this payment
    bool paymentProcessed = false;
    for (int i = 0; i < totalSlots; i++) {
      // Match by BOTH carId AND slot to ensure we're clearing the right car
      if (cars[i].awaitingPayment && 
          cars[i].carId == firstPayment.carId && 
          (i + 1) == firstPayment.slot) {
        
        Serial.printf("✅ Found matching car in slot %d\n", i + 1);
        Serial.printf("   Clearing: Car #%lu, Cost: %lu tk\n", cars[i].carId, cars[i].cost);
        
        // Clear ONLY this specific car's data
        cars[i].awaitingPayment = false;
        cars[i].carId = 0;
        cars[i].cost = 0;
        cars[i].duration = 0;
        cars[i].entryTime = 0;
        cars[i].exitTime = 0;
        
        paymentProcessed = true;
        Serial.println("✅ Payment processed and slot cleared");
        break; // Important: Stop after finding the correct car
      }
    }
    
    if (paymentProcessed) {
      // UNBLOCK the slot so it can be reused
      int slotIndex = firstPayment.slot - 1; // Convert to 0-based index
      slotBlocked[slotIndex] = false;
      Serial.printf("🔓 Slot %d is now UNBLOCKED and available for reuse\n", firstPayment.slot);
      
      // Remove the processed payment from queue
      removeFromPaymentQueue(firstPayment.carId);
      Serial.printf("📤 Removed payment from queue. Remaining: %d\n", paymentQueueSize);
      
      // Mark payment as processed for this exit event
      exitPaymentProcessed = true;
      Serial.println("🔒 Payment processing locked until next exit event");
    } else {
      Serial.println("⚠️ Warning: Payment in queue but no matching car found!");
    }
    
    Serial.println("==========================================\n");
  }
  
  // Update previous exit state
  lastExitActive = exitActive;

  // SEPARATE GATE LOGIC FOR ENTRY AND EXIT
  
  // EXIT GATE: Only controlled by exit IR sensor
  bool exitCondition = exitActive;
  
  // ENTRY GATE: Controlled by entry IR + camera + available slots
  // ALL three conditions MUST be true simultaneously:
  // 1. IR sensor triggered (car physically present at entry)
  // 2. Camera detected and verified it's a car (within timeout period)
  // 3. There are available slots
  bool entryCondition = (entryActive && camCarDetected && availableSlots > 0);

  // Debug logging to help diagnose issues
  static unsigned long lastDebugLog = 0;
  if (millis() - lastDebugLog > 1000) { // Log every second
    Serial.println("=== Gate Logic Status ===");
    Serial.printf("Entry IR: %s | Exit IR: %s | Camera: %s | Available Slots: %d\n", 
                  entryActive ? "TRIGGERED" : "clear", 
                  exitActive ? "TRIGGERED" : "clear",
                  camCarDetected ? "CAR DETECTED" : "no car",
                  availableSlots);
    Serial.printf("Entry Condition: %s | Exit Condition: %s\n",
                  entryCondition ? "TRUE" : "FALSE",
                  exitCondition ? "TRUE" : "FALSE");
    Serial.printf("Entry Gate: %s (angle: %d, target: %d)\n", entryGateStatus.c_str(), entryCurrentAngle, entryTargetAngle);
    Serial.printf("Exit Gate: %s (angle: %d, target: %d)\n", exitGateStatus.c_str(), exitCurrentAngle, exitTargetAngle);
    Serial.printf("Payment Queue Size: %d\n", paymentQueueSize);
    
    // Show current car status and slot blocking status
    for (int i = 0; i < totalSlots; i++) {
      if (cars[i].carId > 0 || slotBlocked[i]) {
        String status = cars[i].present ? "Present" : (cars[i].awaitingPayment ? "Awaiting Payment" : "Empty");
        String blocked = slotBlocked[i] ? " [🔒 BLOCKED]" : "";
        Serial.printf("  Slot %d: Car #%lu - %s%s, Cost: %lu tk\n", 
                      i + 1, 
                      cars[i].carId, 
                      status.c_str(),
                      blocked.c_str(),
                      cars[i].cost);
      }
    }
    
    Serial.println("========================");
    lastDebugLog = millis();
  }

  // ENTRY GATE CONTROL (ONLY for entry, NEVER for exit)
  if (entryCondition && entryTargetAngle != entryServoOpen) {
    Serial.println("🚪 Opening ENTRY gate...");
    Serial.println("  ✅ Reason: ENTRY (IR + Camera + Available Slot)");
    Serial.println("  🎯 All entry conditions met simultaneously!");
    requestEntryServo(entryServoOpen);
    gateOpenedForEntry = true;
  } else if (!entryCondition && entryTargetAngle != entryServoClosed) {
    Serial.println("🚪 Closing ENTRY gate...");
    
    // Clear camera detection flag after gate closes following an entry
    if (gateOpenedForEntry && !entryActive) {
      Serial.println("  🧹 Clearing camera detection flag (entry complete)");
      camCarDetected = false;
      gateOpenedForEntry = false;
    }
    
    requestEntryServo(entryServoClosed);
  }
  
  // If entry gate is fully open and entry IR is no longer triggered, prepare for next car
  if (entryGateStatus == "Open" && !entryActive && gateOpenedForEntry) {
    Serial.println("  🚗 Car passed through entry - ready for gate close");
  }

  // EXIT GATE CONTROL (ONLY for exit, NEVER for entry)
  if (exitCondition && exitTargetAngle != exitServoOpen) {
    Serial.println("🚪 Opening EXIT gate...");
    Serial.println("  ✅ Reason: EXIT (IR triggered)");
    requestExitServo(exitServoOpen);
  } else if (!exitCondition && exitTargetAngle != exitServoClosed) {
    Serial.println("🚪 Closing EXIT gate...");
    requestExitServo(exitServoClosed);
  }
  
  // When exit IR clears after payment, system is ready for next exit
  if (!exitActive && lastExitActive && exitPaymentProcessed) {
    Serial.println("  ✅ Exit complete - ready for next car to exit");
  }
}

// ---------------- HTTP Handlers ----------------
String localIPStr() {
  IPAddress ip = WiFi.localIP();
  if (ip[0] == 0) return String("");
  return ip.toString();
}

void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleStatus() {
  String json = "{";
  json += "\"total\":" + String(totalSlots) + ",";
  json += "\"available\":" + String(availableSlots) + ",";
  json += "\"entryGate\":\"" + entryGateStatus + "\",";
  json += "\"exitGate\":\"" + exitGateStatus + "\",";
  json += "\"manual\":"; json += manualOverride ? "true" : "false"; json += ",";
  json += "\"entryAngle\":" + String(entryCurrentAngle) + ",";
  json += "\"exitAngle\":" + String(exitCurrentAngle) + ",";
  json += "\"entryTarget\":" + String(entryTargetAngle) + ",";
  json += "\"exitTarget\":" + String(exitTargetAngle) + ",";
  json += "\"slots\":[";
  for (int i = 0; i < totalSlots; i++) {
    if (i > 0) json += ",";
    json += slotOccupied[i] ? "true" : "false";
  }
  json += "],";
  json += "\"ir\":{";
  json += "\"entry\":" + String(sensors.entryRaw) + ",";
  json += "\"exit\":"  + String(sensors.exitRaw)  + ",";
  json += "\"slot1\":" + String(sensors.slot1Raw) + ",";
  json += "\"slot2\":" + String(sensors.slot2Raw) + ",";
  json += "\"slot3\":" + String(sensors.slot3Raw);
  json += "},";
  json += "\"camera\":{";
  json += "\"carDetected\":"; json += camCarDetected ? "true" : "false"; json += ",";
  json += "\"lastDetection\":" + String(lastCamDetection);
  json += "},";
  json += "\"cars\":[";
  for (int i = 0; i < totalSlots; i++) {
    if (i > 0) json += ",";
    json += "{";
    json += "\"slot\":" + String(i+1) + ",";
    json += "\"carId\":" + String(cars[i].carId) + ",";
    json += "\"present\":"; json += cars[i].present ? "true" : "false"; json += ",";
    json += "\"duration\":" + String(cars[i].duration) + ",";
    json += "\"cost\":" + String(cars[i].cost) + ",";
    json += "\"awaitingPayment\":"; json += cars[i].awaitingPayment ? "true" : "false";
    json += "}";
  }
  json += "],";
  json += "\"paymentQueue\":[";
  for (int i = 0; i < paymentQueueSize; i++) {
    if (i > 0) json += ",";
    json += "{";
    json += "\"carId\":" + String(paymentQueue[i].carId) + ",";
    json += "\"cost\":" + String(paymentQueue[i].cost) + ",";
    json += "\"slot\":" + String(paymentQueue[i].slot) + ",";
    json += "\"exitTime\":" + String(paymentQueue[i].exitTime) + ",";
    json += "\"priority\":" + String(i + 1); // 1 = highest priority
    json += "}";
  }
  json += "],";
  json += "\"paymentQueueSize\":" + String(paymentQueueSize) + ",";
  json += "\"ip\":\"" + localIPStr() + "\",";
  json += "\"mdns\":\"http://" + String(MDNS_NAME) + ".local\"";
  json += "}";
  server.send(200, "application/json", json);
}

void handleManual() {
  String enable = server.hasArg("enable") ? server.arg("enable") : "";
  if (enable == "1" || enable == "true") {
    manualOverride = true;
    Serial.println("\n⚠️ ========== MANUAL MODE ENABLED ==========");
    Serial.println("⚠️ Automatic detection is DISABLED");
    Serial.println("⚠️ Use Open/Close buttons to control gate");
    Serial.println("⚠️ Switch back to Auto Mode for normal operation");
    Serial.println("==========================================\n");
  } else if (enable == "0" || enable == "false") {
    manualOverride = false;
    Serial.println("\n✅ ========== AUTO MODE ENABLED ==========");
    Serial.println("✅ Automatic detection is ACTIVE");
    Serial.println("✅ Gate will respond to sensors and camera");
    Serial.println("==========================================\n");
  }
  server.send(200, "text/plain", "OK");
}

void handleOpen() {
  // Only allow manual control if already in manual mode
  if (manualOverride) {
    requestEntryServo(entryServoOpen);
    requestExitServo(exitServoOpen);
    server.send(200, "text/plain", "OK - Both gates opening");
  } else {
    server.send(403, "text/plain", "Switch to Manual Mode first");
  }
}

void handleClose() {
  // Only allow manual control if already in manual mode
  if (manualOverride) {
    requestEntryServo(entryServoClosed);
    requestExitServo(exitServoClosed);
    server.send(200, "text/plain", "OK - Both gates closing");
  } else {
    server.send(403, "text/plain", "Switch to Manual Mode first");
  }
}

void handleCarDetected() {
  Serial.println("\n🚗 ========== Car Detection Endpoint Called ==========");
  
  if (server.hasArg("plain")) {
    String body = server.arg("plain");
    Serial.println("📨 Received payload: " + body);
    
    // Parse the JSON to extract confidence (basic parsing)
    int confIndex = body.indexOf("\"confidence\":");
    if (confIndex != -1) {
      int startIndex = confIndex + 13; // length of "confidence":
      int endIndex = body.indexOf(',', startIndex);
      if (endIndex == -1) endIndex = body.indexOf('}', startIndex);
      
      String confStr = body.substring(startIndex, endIndex);
      float confidence = confStr.toFloat();
      
      Serial.printf("🎯 Car detected with %.2f%% confidence\n", confidence * 100);
      Serial.printf("📍 Current status - Entry IR: %s, Available Slots: %d\n", 
                    entryActive ? "TRIGGERED" : "clear", availableSlots);
      
      // Update camera detection status
      camCarDetected = true;
      lastCamDetection = millis();
      
      Serial.println("✅ Camera detection flag SET");
      Serial.println("================================================\n");
      
      server.send(200, "application/json", "{\"status\":\"received\",\"message\":\"Car detection processed\"}");
    } else {
      Serial.println("❌ Invalid JSON format - no confidence field found");
      server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Invalid JSON format\"}");
    }
  } else {
    Serial.println("❌ No data received in request body");
    server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"No data received\"}");
  }
}

void handleNotFound() {
  if (server.uri() == "/favicon.ico") {
    server.send(204);
    return;
  }
  server.send(404, "text/plain", "Not found");
}

// ---------------- Setup/Loop ----------------
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== ESP32 Parking System Starting ===");
  
  // GPIO
  pinMode(IR_ENTRY, INPUT);
  pinMode(IR_EXIT, INPUT);
  pinMode(IR_SLOT1, INPUT);
  pinMode(IR_SLOT2, INPUT);
  pinMode(IR_SLOT3, INPUT);
  Serial.println("📍 GPIO pins configured (3 slots)");

  // Servos
  entryServo.attach(SERVO_ENTRY_PIN);
  entryServo.write(entryCurrentAngle);
  Serial.println("🔧 Entry servo initialized on pin " + String(SERVO_ENTRY_PIN));
  
  exitServo.attach(SERVO_EXIT_PIN);
  exitServo.write(exitCurrentAngle);
  Serial.println("🔧 Exit servo initialized on pin " + String(SERVO_EXIT_PIN));

  // WiFi STA
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("📶 Connecting to WiFi");

  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000UL) {
    delay(200);
    Serial.print(".");
  }

  // Fallback AP if STA fails
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\n⚠️  WiFi failed, starting AP mode");
    WiFi.mode(WIFI_AP);
    WiFi.softAP("ESP32-Parking", "parking123");
    Serial.println("📡 AP: ESP32-Parking");
  } else {
    Serial.println();
    Serial.print("✅ WiFi connected! IP: ");
    Serial.println(WiFi.localIP());
  }

  // mDNS (best-effort)
  if (MDNS.begin(MDNS_NAME)) {
    Serial.print("🌐 mDNS: http://");
    Serial.print(MDNS_NAME);
    Serial.println(".local");
  }

  // Web routes
  server.on("/", HTTP_GET, handleRoot);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/api/manual", HTTP_POST, handleManual);
  server.on("/api/open", HTTP_POST, handleOpen);
  server.on("/api/close", HTTP_POST, handleClose);
  server.on("/api/car_detected", HTTP_POST, handleCarDetected);
  server.onNotFound(handleNotFound);
  server.begin();
  
  Serial.println("🌐 Web server started");
  Serial.println("📸 Ready to receive ESP32-CAM car detections");
  Serial.println("=== System Ready ===\n");
}


void loop() {
  server.handleClient();
  readSensorsAndUpdateSlots();
  autoGateLogic();
  updateServos();
}
