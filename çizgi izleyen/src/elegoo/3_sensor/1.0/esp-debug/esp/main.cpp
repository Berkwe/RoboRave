#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

const char* ssid = "";
const char* password = "";

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

#define RXD1 3
#define TXD1 40

String buffer = "";

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, 
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if(type == WS_EVT_CONNECT){
    Serial.printf("WebSocket #%u baglandi\n", client->id());
  } else if(type == WS_EVT_DISCONNECT){
    Serial.printf("WebSocket #%u ayrildi\n", client->id());
  } else if(type == WS_EVT_DATA) {
    // Web'den gelen komutları Arduino'ya prefix ile ilet
    String msg = "";
    for(size_t i = 0; i < len; i++) {
      msg += (char)data[i];
    }
    // Prefix ekleyerek gönder
    Serial1.println("ESP:" + msg);
    Serial.println("Komut gonderildi: ESP:" + msg);
  }
}

String getHTML() {
  return R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Elegoo Smart Car Monitor</title>
  <style>
    * { margin: 0; padding: 0; box-sizing: border-box; }
    body { 
      background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
      font-family: 'Courier New', monospace;
      padding: 20px;
      min-height: 100vh;
    }
    .container {
      max-width: 1400px;
      margin: 0 auto;
      background: rgba(0, 0, 0, 0.85);
      border-radius: 15px;
      padding: 20px;
      box-shadow: 0 8px 32px rgba(0, 0, 0, 0.3);
    }
    h1 { 
      color: #00ff41;
      text-align: center;
      margin-bottom: 20px;
      text-shadow: 0 0 10px #00ff41;
    }
    .main-grid {
      display: grid;
      grid-template-columns: 1fr 350px;
      gap: 15px;
      margin-bottom: 15px;
    }
    @media (max-width: 1024px) {
      .main-grid { grid-template-columns: 1fr; }
    }
    .status {
      display: flex;
      justify-content: space-around;
      margin-bottom: 15px;
      padding: 10px;
      background: rgba(0, 255, 65, 0.1);
      border-radius: 8px;
      border: 1px solid #00ff41;
      flex-wrap: wrap;
    }
    .status-item {
      color: #00ff41;
      font-size: 13px;
      margin: 5px;
    }
    .status-value {
      color: #fff;
      font-weight: bold;
    }
    #monitor { 
      background: #000;
      color: #00ff41;
      border: 2px solid #00ff41;
      border-radius: 8px;
      padding: 15px;
      white-space: pre;
      height: 500px;
      overflow-y: auto;
      font-size: 12px;
      line-height: 1.3;
    }
    #monitor::-webkit-scrollbar { width: 10px; }
    #monitor::-webkit-scrollbar-track { background: #1a1a1a; }
    #monitor::-webkit-scrollbar-thumb { background: #00ff41; border-radius: 5px; }
    
    .control-panel {
      background: rgba(0, 255, 65, 0.05);
      border: 2px solid #00ff41;
      border-radius: 8px;
      padding: 15px;
      height: 500px;
      overflow-y: auto;
    }
    .control-panel h2 {
      color: #00ff41;
      font-size: 16px;
      margin-bottom: 15px;
      text-align: center;
    }
    .control-section {
      margin-bottom: 20px;
      padding: 15px;
      background: rgba(0, 0, 0, 0.3);
      border-radius: 5px;
      border: 1px solid rgba(0, 255, 65, 0.3);
    }
    .control-section h3 {
      color: #00ff41;
      font-size: 14px;
      margin-bottom: 10px;
    }
    .input-group {
      margin-bottom: 10px;
    }
    .input-group label {
      display: block;
      color: #00ff41;
      font-size: 12px;
      margin-bottom: 5px;
    }
    .input-group input {
      width: 100%;
      background: #000;
      border: 1px solid #00ff41;
      color: #00ff41;
      padding: 8px;
      border-radius: 4px;
      font-family: 'Courier New', monospace;
    }
    .input-group input:focus {
      outline: none;
      border-color: #00cc33;
      box-shadow: 0 0 5px #00ff41;
    }
    
    .controls {
      margin-top: 15px;
      display: flex;
      gap: 10px;
      justify-content: center;
      flex-wrap: wrap;
    }
    button {
      background: #00ff41;
      color: #000;
      border: none;
      padding: 10px 20px;
      border-radius: 5px;
      cursor: pointer;
      font-family: 'Courier New', monospace;
      font-weight: bold;
      transition: all 0.3s;
      font-size: 13px;
    }
    button:hover {
      background: #00cc33;
      transform: translateY(-2px);
    }
    button:active {
      transform: translateY(0);
    }
    .btn-stop {
      background: #ff4444;
      color: #fff;
    }
    .btn-stop:hover {
      background: #cc0000;
    }
    .btn-start {
      background: #44ff44;
    }
    .btn-start:hover {
      background: #00cc33;
    }
    .btn-full {
      width: 100%;
      margin-bottom: 10px;
    }
    .connection-status {
      display: inline-block;
      width: 12px;
      height: 12px;
      border-radius: 50%;
      margin-left: 10px;
      animation: pulse 2s infinite;
    }
    .connected { background: #00ff41; }
    .disconnected { background: #ff0000; }
    @keyframes pulse {
      0%, 100% { opacity: 1; }
      50% { opacity: 0.5; }
    }
    .info-text {
      color: #888;
      font-size: 11px;
      margin-top: 5px;
    }
  </style>
</head>
<body>
  <div class="container">
    <h1>🤖 ELEGOO SMART CAR V4</h1>
    <div class="status">
      <div class="status-item">
        Bağlantı: <span class="status-value" id="status">Bağlanıyor...</span>
        <span class="connection-status disconnected" id="indicator"></span>
      </div>
      <div class="status-item">
        Veri: <span class="status-value" id="count">0</span>
      </div>
      <div class="status-item">
        Güncelleme: <span class="status-value" id="time">-</span>
      </div>
    </div>
    
    <div class="main-grid">
      <div>
        <div id="monitor">Bağlantı bekleniyor...</div>
        <div class="controls">
          <button onclick="clearMonitor()">🗑️ Temizle</button>
          <button onclick="toggleScroll()">📜 Kaydırma: <span id="scroll">AÇIK</span></button>
          <button onclick="downloadLog()">💾 Kaydet</button>
        </div>
      </div>
      
      <div class="control-panel">
        <h2>⚙️ KONTROL PANELİ</h2>
        
        <div class="control-section">
          <h3>🎮 Robot Kontrolü</h3>
          <button class="btn-start btn-full" onclick="sendCommand('START')">▶️ BAŞLAT</button>
          <button class="btn-stop btn-full" onclick="sendCommand('STOP')">⏹️ DURDUR</button>
        </div>
        
        <div class="control-section">
          <h3>🎛️ PID Katsayıları</h3>
          <div class="input-group">
            <label>Kp (Oransal)</label>
            <input type="number" id="kp" value="90.0" step="0.1">
          </div>
          <div class="input-group">
            <label>Ki (İntegral)</label>
            <input type="number" id="ki" value="0.16" step="0.01">
          </div>
          <div class="input-group">
            <label>Kd (Türev)</label>
            <input type="number" id="kd" value="24.0" step="0.1">
          </div>
          <div class="input-group">
            <label>Temel Hız</label>
            <input type="number" id="baseSpeed" value="50" step="1" min="0" max="130">
          </div>
          <button class="btn-full" onclick="sendPID()">📤 PID Gönder</button>
          <div class="info-text">Değerleri değiştirip "PID Gönder" butonuna basın</div>
        </div>
        
        <div class="control-section">
          <h3>🔄 Hızlı Komutlar</h3>
          <button class="btn-full" onclick="sendCommand('CALIBRATE')">📏 Kalibrasyon</button>
          <button class="btn-full" onclick="sendCommand('RESET')">🔄 Sıfırla</button>
          <button class="btn-full" onclick="sendCommand('STATUS')">📊 Durum</button>
        </div>
      </div>
    </div>
  </div>
  
  <script>
    const monitor = document.getElementById('monitor');
    const indicator = document.getElementById('indicator');
    const status = document.getElementById('status');
    const count = document.getElementById('count');
    const time = document.getElementById('time');
    let autoScroll = true;
    let msgCount = 0;
    let ws;
    
    function connect() {
      ws = new WebSocket('ws://' + location.hostname + '/ws');
      ws.onopen = function() {
        status.textContent = 'Bağlı';
        indicator.className = 'connection-status connected';
        monitor.textContent = 'Bağlantı kuruldu. Veri bekleniyor...\n';
      };
      ws.onmessage = function(event) {
        monitor.textContent += event.data;
        msgCount++;
        count.textContent = msgCount;
        time.textContent = new Date().toLocaleTimeString('tr-TR');
        if (autoScroll) monitor.scrollTop = monitor.scrollHeight;
      };
      ws.onerror = function() {
        status.textContent = 'Hata';
        indicator.className = 'connection-status disconnected';
      };
      ws.onclose = function() {
        status.textContent = 'Bağlantı Kesildi';
        indicator.className = 'connection-status disconnected';
        setTimeout(connect, 3000);
      };
    }
    
    function sendCommand(cmd) {
      if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(cmd);
        monitor.textContent += `[KOMUT GÖNDER] ${cmd}\n`;
        if (autoScroll) monitor.scrollTop = monitor.scrollHeight;
      } else {
        alert('WebSocket bağlantısı yok!');
      }
    }
    
    function sendPID() {
      const kp = document.getElementById('kp').value;
      const ki = document.getElementById('ki').value;
      const kd = document.getElementById('kd').value;
      const baseSpeed = document.getElementById('baseSpeed').value;
      
      const cmd = `PID:${kp},${ki},${kd},${baseSpeed}`;
      sendCommand(cmd);
    }
    
    function clearMonitor() {
      monitor.textContent = '';
      msgCount = 0;
      count.textContent = '0';
    }
    
    function toggleScroll() {
      autoScroll = !autoScroll;
      document.getElementById('scroll').textContent = autoScroll ? 'AÇIK' : 'KAPALI';
    }
    
    function downloadLog() {
      const blob = new Blob([monitor.textContent], {type: 'text/plain'});
      const url = URL.createObjectURL(blob);
      const a = document.createElement('a');
      a.href = url;
      a.download = 'smartcar_' + Date.now() + '.txt';
      a.click();
    }
    
    connect();
  </script>
</body>
</html>
)rawliteral";
}


void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, RXD1, TXD1);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println("\nWiFi Baglandi: " + WiFi.localIP().toString());

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/html", getHTML());
  });
  server.begin();
  Serial.println("Web server basladi.");
}

void loop() {
  while (Serial1.available()) {
    char c = Serial1.read();
    buffer += c;
    if(c == '\n'){
      if (!buffer.startsWith("ESP:")) {
      ws.textAll(buffer);

      }
      buffer = "";
    }
  }
  ws.cleanupClients();
}