# lgpio Verwendung mit Remote-Daemon (rgpiod)

## Voraussetzungen

### Auf dem Host (Raspberry Pi)
```bash
# lgpio installieren
sudo apt install lgpio

# rgpiod Daemon starten
sudo rgpiod

# Oder als systemd Service
sudo systemctl enable rgpiod
sudo systemctl start rgpiod
sudo systemctl status rgpiod
```

### Im Docker Container
```bash
# Nur Development-Header installieren
sudo apt install liblgpio-dev
```

**Wichtig:** Der Container braucht **keinen** privilegierten Zugriff, da die Kommunikation über Netzwerk mit dem Daemon erfolgt.

## API-Referenz

### 1. Initialisierung (Remote-Verbindung)

```cpp
#include <lgpio.h>

// Verbindung zum rgpiod auf dem Host (localhost oder IP)
int handle = lgGpiochipOpen(0);  // Chip 0, verbindet automatisch zu rgpiod
if (handle < 0) {
    // Fehler bei der Verbindung
    // handle enthält Fehlercode (negativ)
}
```

**Hinweis:** Falls rgpiod auf einem anderen Host läuft:
```cpp
// Umgebungsvariable setzen vor dem Programmstart
setenv("LG_ADDR", "192.168.1.10", 1);
```

---

### 2. Digital Input konfigurieren

```cpp
int pin = 17;  // GPIO Pin Nummer

// Input mit Pull-Up Resistor
int result = lgGpioClaimInput(handle, LG_SET_PULL_UP, pin);

// Alternativen:
// Pull-Down Resistor
int result = lgGpioClaimInput(handle, LG_SET_PULL_DOWN, pin);

// Keine Pull-Resistoren
int result = lgGpioClaimInput(handle, 0, pin);
```

**Parameter:**
- `handle`: GPIO-Chip Handle (von `lgGpiochipOpen`)
- `flags`: 
  - `LG_SET_PULL_UP` - Pull-Up aktivieren
  - `LG_SET_PULL_DOWN` - Pull-Down aktivieren  
  - `0` - Keine Pull-Resistoren
- `pin`: GPIO Pin Nummer (BCM-Nummerierung)

**Return:** 0 bei Erfolg, negativ bei Fehler

---

### 3. Digital Output konfigurieren

```cpp
int pin = 18;  // GPIO Pin Nummer

// Output mit Initial-Wert 0 (LOW)
int result = lgGpioClaimOutput(handle, 0, pin, 0);

// Output mit Initial-Wert 1 (HIGH)
int result = lgGpioClaimOutput(handle, 0, pin, 1);
```

**Parameter:**
- `handle`: GPIO-Chip Handle
- `flags`: 0 (keine speziellen Flags)
- `pin`: GPIO Pin Nummer
- `level`: Initial-Wert (0 = LOW, 1 = HIGH)

**Return:** 0 bei Erfolg, negativ bei Fehler

---

### 4. PWM konfigurieren

```cpp
int pin = 18;            // GPIO Pin (muss PWM-fähig sein)
float frequency = 1000;  // Frequenz in Hz (z.B. 1 kHz)
float dutycycle = 50.0;  // Duty Cycle in % (0.0 - 100.0)
int offset = 0;          // Phase Offset in Mikrosekunden
int cycles = 0;          // 0 = kontinuierlich

// PWM starten
int result = lgTxPwm(handle, pin, frequency, dutycycle, offset, cycles);
```

**Parameter:**
- `handle`: GPIO-Chip Handle
- `pin`: GPIO Pin Nummer (muss PWM-fähig sein)
- `frequency`: Frequenz in Hz
- `dutycycle`: Duty Cycle in Prozent (0.0 - 100.0)
  - 0.0 = immer LOW
  - 50.0 = 50% HIGH, 50% LOW
  - 100.0 = immer HIGH
- `offset`: Phase Offset in Mikrosekunden (normalerweise 0)
- `cycles`: Anzahl Zyklen (0 = kontinuierlich)

**Return:** Waveform-ID bei Erfolg, negativ bei Fehler

**PWM-fähige Pins auf Raspberry Pi:**
- Hardware PWM (bevorzugt): GPIO 12, 13, 18, 19
- Software PWM: Alle GPIOs möglich, aber weniger präzise

---

### 5. Werte lesen (Input)

```cpp
int value = lgGpioRead(handle, pin);
if (value < 0) {
    // Fehler aufgetreten
} else {
    // value ist 0 (LOW) oder 1 (HIGH)
}
```

**Return:** 
- `0` = LOW
- `1` = HIGH
- Negativ = Fehler

---

### 6. Werte schreiben (Output)

```cpp
// HIGH schreiben
int result = lgGpioWrite(handle, pin, 1);

// LOW schreiben  
int result = lgGpioWrite(handle, pin, 0);
```

**Return:** 0 bei Erfolg, negativ bei Fehler

---

### 7. PWM Duty Cycle ändern (zur Laufzeit)

```cpp
// Duty Cycle auf 75% ändern
float new_dutycycle = 75.0;
lgTxPwm(handle, pin, frequency, new_dutycycle, 0, 0);
```

**Hinweis:** Die Frequenz sollte gleich bleiben, nur der Duty Cycle wird angepasst.

---

### 8. Aufräumen

```cpp
// Einzelnen Pin freigeben
lgGpioFree(handle, pin);

// Chip-Handle schließen (alle Pins werden freigegeben)
lgGpiochipClose(handle);
```

**Wichtig:** Immer aufräumen, um Ressourcen freizugeben!

---

## Fehlerbehandlung

Alle lgpio-Funktionen geben bei Fehler einen **negativen Wert** zurück:

```cpp
int result = lgGpioClaimOutput(handle, 0, pin, 0);
if (result < 0) {
    switch (result) {
        case LG_BAD_HANDLE:
            // Ungültiger Handle
            break;
        case LG_BAD_GPIO:
            // Ungültige GPIO Pin Nummer
            break;
        case LG_GPIO_IN_USE:
            // GPIO bereits in Verwendung
            break;
        default:
            // Anderer Fehler
            break;
    }
}
```

Häufige Fehlercodes:
- `LG_BAD_HANDLE` (-25): Handle ungültig
- `LG_BAD_GPIO` (-28): Pin-Nummer ungültig
- `LG_GPIO_IN_USE` (-50): Pin bereits belegt
- `LG_CANNOT_OPEN_CHIP` (-1): Kann Chip nicht öffnen (rgpiod läuft nicht?)

---

## Beispiel: Vollständige Verwendung

```cpp
#include <lgpio.h>
#include <iostream>
#include <unistd.h>

int main() {
    // Verbindung zu rgpiod herstellen
    int handle = lgGpiochipOpen(0);
    if (handle < 0) {
        std::cerr << "Fehler beim Öffnen: " << handle << std::endl;
        return 1;
    }

    // LED auf GPIO 18 konfigurieren
    int led_pin = 18;
    lgGpioClaimOutput(handle, 0, led_pin, 0);

    // Button auf GPIO 17 mit Pull-Up konfigurieren
    int button_pin = 17;
    lgGpioClaimInput(handle, LG_SET_PULL_UP, button_pin);

    // PWM auf GPIO 12 starten (1 kHz, 50%)
    int pwm_pin = 12;
    lgTxPwm(handle, pwm_pin, 1000, 50.0, 0, 0);

    // Loop
    for (int i = 0; i < 100; i++) {
        // Button lesen
        int button_state = lgGpioRead(handle, button_pin);
        
        // LED entsprechend setzen
        lgGpioWrite(handle, led_pin, button_state);
        
        usleep(100000);  // 100ms warten
    }

    // Aufräumen
    lgGpioFree(handle, led_pin);
    lgGpioFree(handle, button_pin);
    lgGpioFree(handle, pwm_pin);
    lgGpiochipClose(handle);

    return 0;
}
```

---

## Unterschiede zu libgpiod

| Feature | libgpiod | lgpio |
|---------|----------|-------|
| PWM Support | ❌ Nein | ✅ Ja, eingebaut |
| Remote Access | ❌ Nein | ✅ Ja, via rgpiod |
| API-Stil | C++ Objekte | C-API |
| Pi 5 Support | ✅ Ja | ✅ Ja |
| Docker ohne privileges | ❌ Nein (braucht /dev/) | ✅ Ja (mit Daemon) |

---

## Nützliche Links

- **lgpio Dokumentation:** http://abyz.me.uk/lg/lgpio.html
- **rgpiod Dokumentation:** http://abyz.me.uk/lg/rgpiod.html
- **GitHub Repository:** https://github.com/joan2937/lg
