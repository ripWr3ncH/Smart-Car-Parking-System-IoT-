#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ---------------- LCD Setup ----------------
#define LCD_SDA 21
#define LCD_SCL 22
#define LCD_ADDRESS 0x27  // Try 0x3F if 0x27 doesn't work
LiquidCrystal_I2C lcd(LCD_ADDRESS, 16, 2);

// ---------------- WiFi Config ----------------
const char* WIFI_SSID = "sayeed";
const char* WIFI_PASSWORD = "12345678";
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
int entryServoOpen   = 20;       // Adjust for your linkage

// Exit gate servo angles
int exitServoClosed = 100;       // Adjust for your linkage
int exitServoOpen   = 20;        // Adjust for your linkage

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

// ---------------- LCD Display Variables ----------------
bool showingBill = false;       // Track if bill is currently displayed
unsigned long billDisplayStart = 0; // When bill started displaying
const unsigned long billDisplayDuration = 5000; // Show bill for 5 seconds
unsigned long lastLCDUpdate = 0;
const unsigned long lcdUpdateInterval = 500; // Update LCD every 500ms

// ---------------- Web Server ----------------
WebServer server(80);

// ---------------- HTML UI (inline) ----------------
const char INDEX_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Smart Parking</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{font-family:Arial,sans-serif;background:#1a1a2e;color:#eee;padding:10px}
.container{max-width:1000px;margin:0 auto}
h1{text-align:center;padding:20px;background:#16213e;border-radius:10px;margin-bottom:20px}
.sensors{display:grid;grid-template-columns:repeat(3,1fr);gap:10px;margin-bottom:15px}
.sensor{background:#0f3460;padding:15px;border-radius:10px;text-align:center;border:2px solid}
.sensor.active{border-color:#2ea043;background:#1e4d2b}
.sensor.inactive{border-color:#555;background:#2a2a3e}
.sensor-label{font-size:14px;margin-bottom:5px}
.sensor-status{font-size:18px;font-weight:bold}
.status{background:#0f3460;padding:20px;border-radius:10px;margin-bottom:15px;text-align:center}
.slots{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:15px;margin:20px 0}
.slot{padding:30px 20px;border-radius:10px;text-align:center;font-size:18px;font-weight:bold;border:3px solid}
.free{background:#1e4d2b;border-color:#2ea043;color:#2ea043}
.occ{background:#4d1e1e;border-color:#f85149;color:#f85149}
.blocked{background:#4d3b1e;border-color:#d29922;color:#d29922}
.info{background:#0f3460;padding:15px;border-radius:10px;margin-bottom:15px}
table{width:100%;border-collapse:collapse;margin-top:10px}
th,td{padding:10px;text-align:center;border-bottom:1px solid #333}
th{background:#16213e;color:#d29922}
.alert{background:#d29922;color:#000;padding:20px;border-radius:10px;text-align:center;display:none;position:fixed;top:50%;left:50%;transform:translate(-50%,-50%);z-index:1000;min-width:400px;max-width:90vw;box-shadow:0 10px 30px rgba(0,0,0,0.5)}
.alert.show{display:block;animation:pulse 1.5s infinite}
.alert h2{font-size:24px;margin-bottom:15px;border-bottom:2px solid #000;padding-bottom:10px}
.payment-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(180px,1fr));gap:15px;margin:15px 0}
.payment-card{background:rgba(0,0,0,0.2);padding:15px;border-radius:8px;border:2px solid rgba(0,0,0,0.3)}
.payment-card.first{border-color:#ff6b35;background:rgba(255,107,53,0.3)}
.payment-card .amount{font-size:32px;font-weight:bold;margin:5px 0}
.payment-card .details{font-size:14px;margin:3px 0}
.payment-card .badge{display:inline-block;background:#000;color:#d29922;padding:3px 8px;border-radius:4px;font-size:11px;margin-top:5px;font-weight:bold}
@keyframes pulse{0%,100%{transform:translate(-50%,-50%) scale(1)}50%{transform:translate(-50%,-50%) scale(1.05)}}
.overlay{position:fixed;top:0;left:0;width:100%;height:100%;background:rgba(0,0,0,0.7);z-index:999;display:none}
.overlay.show{display:block}
@media(max-width:768px){
h1{font-size:20px;padding:15px}
.sensors{grid-template-columns:1fr}
.slots{grid-template-columns:1fr}
.slot{font-size:16px;padding:20px}
table{font-size:14px}
th,td{padding:8px}
}
</style>
</head>
<body>
<div class="container">
<h1>🚗 Smart Parking System</h1>

<div class="sensors">
<div class="sensor" id="cameraSensor">
<div class="sensor-label">📸 Camera</div>
<div class="sensor-status">--</div>
</div>
<div class="sensor" id="entrySensor">
<div class="sensor-label">🚪 Entry Sensor</div>
<div class="sensor-status">--</div>
</div>
<div class="sensor" id="exitSensor">
<div class="sensor-label">🚪 Exit Sensor</div>
<div class="sensor-status">--</div>
</div>
</div>

<div class="status">
<h2 id="avail">Loading...</h2>
<p id="mode">System Status</p>
</div>

<div class="slots" id="slots"></div>

<div class="info">
<h3>💰 Parking Details</h3>
<table id="carTable">
<thead><tr><th>Slot</th><th>Car</th><th>Status</th><th>Time</th><th>Cost</th></tr></thead>
<tbody><tr><td colspan="5">Loading...</td></tr></tbody>
</table>
</div>
</div>

<div class="overlay" id="overlay"></div>
<div class="alert" id="alert"></div>

<script>
function $(id){return document.getElementById(id)}

async function update(){
try{
const r=await fetch('/status');
const d=await r.json();

if(d.available===0){
  $('avail').textContent='No Slots Available';
}else{
  $('avail').textContent=`${d.available}/${d.total} Slots Available`;
}
$('mode').textContent='✅ Auto Mode Active';

const camera=$('cameraSensor');
const entry=$('entrySensor');
const exit=$('exitSensor');

if(d.camera&&d.camera.carDetected){
camera.className='sensor active';
camera.querySelector('.sensor-status').textContent='✓ Detected';
}else{
camera.className='sensor inactive';
camera.querySelector('.sensor-status').textContent='✗ No Car';
}

if(d.ir&&d.ir.entry===0){
entry.className='sensor active';
entry.querySelector('.sensor-status').textContent='✓ Active';
}else{
entry.className='sensor inactive';
entry.querySelector('.sensor-status').textContent='✗ Clear';
}

if(d.ir&&d.ir.exit===0){
exit.className='sensor active';
exit.querySelector('.sensor-status').textContent='✓ Active';
}else{
exit.className='sensor inactive';
exit.querySelector('.sensor-status').textContent='✗ Clear';
}

const slots=$('slots');
slots.innerHTML='';
d.slots.forEach((occ,i)=>{
const blocked=d.blockedSlots&&d.blockedSlots[i];
const div=document.createElement('div');
div.className='slot '+(blocked?'blocked':occ?'occ':'free');
div.textContent=`SLOT ${i+1}\n${blocked?'🔒 Payment Due':occ?'🚗 Occupied':'✓ Free'}`;
slots.appendChild(div);
});

const tbody=$('carTable').getElementsByTagName('tbody')[0];
tbody.innerHTML='';
if(d.cars&&d.cars.length>0){
d.cars.forEach(car=>{
const row=tbody.insertRow();
row.innerHTML=`
<td>Slot ${car.slot}</td>
<td>${car.carId>0?'#'+car.carId:'-'}</td>
<td>${car.present?'🚗 Present':car.awaitingPayment?'💳 Payment Due':'⬜ Empty'}</td>
<td>${car.duration>0?car.duration+'s':'-'}</td>
<td>${car.cost>0?car.cost+' tk':'-'}</td>
`;
});
}else{
tbody.innerHTML='<tr><td colspan="5">No cars parked</td></tr>';
}

const overlay=$('overlay');
const alert=$('alert');
if(d.paymentQueue&&d.paymentQueue.length>0){
let html='<h2>💳 PAYMENT DUE</h2><div class="payment-grid">';
d.paymentQueue.forEach((p,i)=>{
html+=`<div class="payment-card ${i===0?'first':''}">
<div class="amount">${p.cost} tk</div>
<div class="details">� Car #${p.carId}</div>
<div class="details">📍 Slot ${p.slot}</div>
<div class="badge">${i===0?'⚡ NEXT TO PAY':'Position '+(i+1)}</div>
</div>`;
});
html+='</div><div style="font-size:14px;margin-top:10px">Drive to Exit Gate to Complete Payment</div>';
alert.innerHTML=html;
alert.classList.add('show');
overlay.classList.add('show');
}else{
alert.classList.remove('show');
overlay.classList.remove('show');
}
}catch(e){console.error(e)}
}

setInterval(update,1000);
update();
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

// ---------------- LCD Functions ----------------
void updateLCD() {
  unsigned long now = millis();
  
  // If showing bill, check if it's time to clear it
  if (showingBill && (now - billDisplayStart >= billDisplayDuration)) {
    showingBill = false;
    Serial.println("📺 Bill display timeout - returning to slot status");
  }
  
  // Update LCD at regular intervals (unless showing bill)
  if (!showingBill && (now - lastLCDUpdate < lcdUpdateInterval)) {
    return;
  }
  lastLCDUpdate = now;
  
  lcd.clear();
  
  if (showingBill) {
    // Don't update during bill display to prevent flickering
    return;
  }
  
  // Normal display - show parking availability
  if (availableSlots == 0) {
    // All slots occupied
    lcd.setCursor(0, 0);
    lcd.print("  PARKING FULL  ");
    lcd.setCursor(0, 1);
    lcd.print("   0/3 FREE     ");
  } else {
    // Show available slots
    lcd.setCursor(0, 0);
    lcd.print("SLOTS AVAILABLE:");
    lcd.setCursor(0, 1);
    char buffer[17];
    snprintf(buffer, sizeof(buffer), "Free:%d  Full:%d/%d", 
             availableSlots, 
             totalSlots - availableSlots, 
             totalSlots);
    lcd.print(buffer);
  }
}

void displayBillOnLCD(unsigned long carId, int slot, unsigned long cost, unsigned long duration) {
  lcd.clear();
  
  // Line 1: Car ID and Slot
  lcd.setCursor(0, 0);
  char line1[17];
  snprintf(line1, sizeof(line1), "Car#%lu Slot %d", carId, slot);
  lcd.print(line1);
  
  // Line 2: Cost and Duration
  lcd.setCursor(0, 1);
  char line2[17];
  snprintf(line2, sizeof(line2), "%lutk  %lus", cost, duration);
  lcd.print(line2);
  
  showingBill = true;
  billDisplayStart = millis();
  
  Serial.printf("📺 LCD showing bill: Car #%lu, Slot %d, %lu tk, %lu seconds\n", 
                carId, slot, cost, duration);
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
      // DISPLAY BILL ON LCD
      displayBillOnLCD(firstPayment.carId, firstPayment.slot, firstPayment.cost, 
                      (firstPayment.cost > 0) ? firstPayment.cost : 0); // duration = cost (1tk/sec)
      
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
    Serial.printf("IP Address: %s\n", WiFi.localIP().toString().c_str());
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
  
  // Initialize LCD
 // Wire.begin(LCD_SDA, LCD_SCL);
 //Wire.setClock(100000);
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("ESP32 PARKING");
  lcd.setCursor(0, 1);
  lcd.print("  STARTING...   ");
  Serial.println("📺 LCD initialized (16x2 I2C)");
  delay(2000);
  
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
  
  // Show initial LCD status
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("SLOTS AVAILABLE:");
  lcd.setCursor(0, 1);
  lcd.print("Free:3  Full:0/3");
}


void loop() {
  server.handleClient();
  readSensorsAndUpdateSlots();
  autoGateLogic();
  updateServos();
  updateLCD(); // Update LCD display
}
